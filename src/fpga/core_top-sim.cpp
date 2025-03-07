/*
 * Copyright (C) 2024 Markus Lavin (https://www.zzzconsulting.se/)
 *
 * All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "CLI11.hpp"

#include "Vcore_top.h"
#include "verilated.h"
#include "verilated_fst_c.h"
#include <assert.h>
#include <fstream>
#include <functional>
#include <gtk/gtk.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

static uint64_t g_ticks = 0;
static uint32_t g_frame_idx = 0;

static std::unique_ptr<Vcore_top> dut;

class SimpleUART {
public:
  SimpleUART(const uint8_t &rxd) : rxd_(rxd) {}
  void Tick() {
    switch (state_) {
    case State::Idle:
      if (rxd_ == 0) {
        rx_cntr_ = 0;
        state_ = State::Receive;
      }
      break;
    case State::Receive:
      rx_byte_ = (rx_byte_ >> 1) | (rxd_ << 7);
      if (++rx_cntr_ == 8) {
        fprintf(stderr, "%c", rx_byte_);
        state_ = State::Idle;
      }
      break;
    }
  }

private:
  enum class State { Idle, Receive } state_ = State::Idle;
  const uint8_t &rxd_;
  uint8_t rx_cntr_;
  uint8_t rx_byte_;
};

// sdram, 512mbit 16bit
//
// output wire [12:0] dram_a,
// output wire [ 1:0] dram_ba,
// inout  wire [15:0] dram_dq,
// output wire [ 1:0] dram_dqm,
// output wire        dram_clk,
// output wire        dram_cke,
// output wire        dram_ras_n,
// output wire        dram_cas_n,
// output wire        dram_we_n,
class SimpleSDRAM {
public:
  SimpleSDRAM() {}
  void Tick() {
    if (dut->dram_clk) {
      switch (state_) {
      case State::Idle:
        if (dut->dram_ras_n == 0 && dut->dram_cas_n == 1 &&
            dut->dram_we_n == 1) { // ACTIVATE
          row_ = dut->dram_a & 0x1fff;
          //std::cout << std::hex << "SDRAM: ACTIVATE row: " << row_ << "\n";
        } else if (dut->dram_ras_n == 1 && dut->dram_cas_n == 0 &&
                   dut->dram_we_n == 1) { // READ
          //std::cout << "SDRAM: READ\n";
          ba_ = dut->dram_ba;
          col_ = dut->dram_a & 0x3ff;
          //std::cout << std::hex << "SDRAM: READ row: " << row_
           //         << " col: " << col_ << "\n";
          state_ = State::ReadWait0;
        } else if (dut->dram_ras_n == 1 && dut->dram_cas_n == 0 &&
                   dut->dram_we_n == 0) { // WRITE
          ba_ = dut->dram_ba;
          col_ = dut->dram_a & 0x3ff;
          //std::cout << std::hex << "SDRAM: WRITE row: " << row_
           //         << " col: " << col_ << "\n";
          mem_[Addr(0)] = dut->dram_dq;
          state_ = State::Write1;
        } else if (dut->dram_ras_n == 0 && dut->dram_cas_n == 1 &&
                   dut->dram_we_n == 0) { // PRECHARGE
          //std::cout << "SDRAM: PRECHARGE\n";
        }
        break;
      //
      // Write
      //
      case State::Write1:
        mem_[Addr(1)] = dut->dram_dq; // XXX: Use write mask
        state_ = State::Write2;
        break;
      case State::Write2:
        mem_[Addr(2)] = dut->dram_dq;
        state_ = State::Write3;
        break;
      case State::Write3:
        mem_[Addr(3)] = dut->dram_dq;
        state_ = State::Idle;
        break;
      //
      // Read
      //
      case State::ReadWait0:
        state_ = State::ReadWait1;
        state_ = State::Read0;
        break;
      case State::ReadWait1:
        state_ = State::ReadWait2;
        break;
      case State::ReadWait2:
        state_ = State::Read0;
        break;
      case State::Read0:
        dut->dram_dq = mem_[Addr(0)];
        state_ = State::Read1;
        break;
      case State::Read1:
        dut->dram_dq = mem_[Addr(1)];
        state_ = State::Read2;
        break;
      case State::Read2:
        dut->dram_dq = mem_[Addr(2)];
        state_ = State::Read3;
        break;
      case State::Read3:
        dut->dram_dq = mem_[Addr(3)];
        state_ = State::Idle;
        break;
      }
    }
  }

private:
  enum class State {
    Idle,
    Write1,
    Write2,
    Write3,
    ReadWait0,
    ReadWait1,
    ReadWait2,
    Read0,
    Read1,
    Read2,
    Read3
  } state_ = State::Idle;

  uint32_t Addr(uint32_t offset) {
    uint32_t addr = offset + col_ + (row_ << 10) + (ba_ << 23);
    assert(addr < mem_.size());
    return addr;
  }

  std::array<uint16_t, 32 * 1024 * 1024> mem_;
  uint32_t ba_;  // 2 bits
  uint32_t row_; // 13 bits
  uint32_t col_; // 10 bits
};

class TraceRTL {
public:
  TraceRTL(const std::string &out_path, const std::vector<std::string> &modules,
           uint32_t begin_frame)
      : begin_frame_(begin_frame) {
    trace = new VerilatedFstC;
    trace->set_time_unit("1ps");
    trace->set_time_resolution("1ps");
    for (auto &module : modules) {
      trace->dumpvars(1, module);
    }
    dut->trace(trace, 99);
    trace->open(out_path.c_str());
  }
  void Tick() {
    if (g_frame_idx >= begin_frame_) {
      trace->dump(g_ticks);
      if (g_frame_idx > last_flush_frame_) {
        trace->flush();
        last_flush_frame_ = g_frame_idx;
      }
    }
  }

private:
  VerilatedFstC *trace;
  uint32_t begin_frame_;
  uint32_t last_flush_frame_ = 0;
};

class FrameDumper {
public:
  FrameDumper() {
    m_FramePixBuf =
        gdk_pixbuf_new(GDK_COLORSPACE_RGB, FALSE, 8, c_Xres, c_Yres);
    gdk_pixbuf_fill(m_FramePixBuf, 0);
  }
  void Tick() {
    bool FrameDone = false;

    if (dut->video_hs) {
      m_HCntr = 0;
      m_VCntr++;
    }
    if (dut->video_vs) {
      m_VCntr = 0;
      FrameDone = true;
    }

    unsigned m_HCntrShifted = m_HCntr - 70;
    unsigned m_VCntrShifted = m_VCntr - 10;
    if (0 <= m_HCntrShifted && m_HCntrShifted < c_Xres && 0 <= m_VCntrShifted &&
        m_VCntrShifted < c_Yres) {
      guchar Red = dut->video_rgb >> 16;
      guchar Green = dut->video_rgb >> 8;
      guchar Blue = dut->video_rgb & 0xff;
      PutPixel(m_FramePixBuf, m_HCntrShifted, m_VCntrShifted, Red, Green, Blue);
    }

    m_HCntr++;

    if (FrameDone) {
      char buf[32];
      snprintf(buf, sizeof(buf), "vicii-%04d.png", g_frame_idx);
      gdk_pixbuf_save(m_FramePixBuf, buf, "png", NULL, NULL);
      printf("%s\n", buf);
    }
  }

private:
  void PutPixel(GdkPixbuf *pixbuf, int x, int y, guchar red, guchar green,
                guchar blue) {
    int width, height, rowstride, n_channels;
    guchar *pixels, *p;

    n_channels = gdk_pixbuf_get_n_channels(pixbuf);

    g_assert(gdk_pixbuf_get_colorspace(pixbuf) == GDK_COLORSPACE_RGB);
    g_assert(gdk_pixbuf_get_bits_per_sample(pixbuf) == 8);
    g_assert(!gdk_pixbuf_get_has_alpha(pixbuf));
    g_assert(n_channels == 3);

    width = gdk_pixbuf_get_width(pixbuf);
    height = gdk_pixbuf_get_height(pixbuf);

    g_assert(x >= 0 && x < width);
    g_assert(y >= 0 && y < height);

    rowstride = gdk_pixbuf_get_rowstride(pixbuf);
    pixels = gdk_pixbuf_get_pixels(pixbuf);

    p = pixels + y * rowstride + x * n_channels;
    p[0] = red;
    p[1] = green;
    p[2] = blue;
  }

  const unsigned c_Xres = 504;
  const unsigned c_Yres = 312;
  GdkPixbuf *m_FramePixBuf;
  unsigned m_HCntr = 0;
  unsigned m_VCntr = 0;
};

class BridgeHandler {
public:
  void Tick() {
    dut->bridge_addr = 0;
    dut->bridge_rd = 0;
    dut->bridge_wr = 0;
    dut->bridge_wr_data = 0;

    switch (bridge_state) {
    case 0: // Wait for reset to release
      if (dut->reset_n) {
        cntr = 0;
        ds_it = dataslots.begin();
        bridge_state = 100;
      }
      break;
    case 100: // Write Data Slot Size table (slot id)
      if (ds_it == dataslots.end()) {
        bridge_state = 1;
      } else {
        auto &dse = *ds_it;
        dut->bridge_addr = 0xf8002000 + cntr * 8 + 0;
        dut->bridge_wr = 1;
        dut->bridge_wr_data = dse.first;
        bridge_state = 101;
      }
      break;
    case 101: { // Write Data Slot Size table (size)
      auto &dse = *ds_it;
      dut->bridge_addr = 0xf8002000 + cntr * 8 + 4;
      dut->bridge_wr = 1;
      dut->bridge_wr_data = dse.second.second;
      cntr++;
      ds_it++;
      bridge_state = 100;
      break;
    }
    case 200: // Data slot update
      dut->bridge_addr = 0xf8000020;
      dut->bridge_wr = 1;
      dut->bridge_wr_data = *updated_dataslots_iter++; // slot id
      bridge_state = 201;
      break;
    case 201: // Data slot update
      dut->bridge_addr = 0xf8000000;
      dut->bridge_wr = 1;
      dut->bridge_wr_data = 0x434d008a;
      bridge_state = 202;
      break;
    case 202: // Data slot update
      if (updated_dataslots_iter != updated_dataslots.end()) {
        bridge_state = 200;
      } else {
        bridge_state = 2;
      }
      break;
    case 1: // Write status OK
      dut->bridge_addr = 0xf8001000;
      dut->bridge_wr = 1;
      dut->bridge_wr_data = 0x6f6b1234;
      if (updated_dataslots_iter != updated_dataslots.end()) {
        bridge_state = 200;
      } else {
        bridge_state = 2;
      }
      break;
    case 2: // Wait for data-slot-read command
      dut->bridge_addr = 0xf8001000;
      dut->bridge_rd = 1;
      if (dut->bridge_rd_data == 0x636D0180) {
        dut->bridge_addr = 0xf8001020;
        bridge_state = 3;
      }
      break;
    case 3: // Latch slot_id
      ds_read_slot_id = dut->bridge_rd_data;
      dut->bridge_addr = 0xf8001024;
      dut->bridge_rd = 1;
      bridge_state = 4;
      break;
    case 4: // Latch slot_offset
      ds_read_slot_offset = dut->bridge_rd_data;
      dut->bridge_addr = 0xf8001028;
      dut->bridge_rd = 1;
      bridge_state = 5;
      break;
    case 5: // Latch bridge address
      ds_read_bridge_address = dut->bridge_rd_data;
      dut->bridge_addr = 0xf800102c;
      dut->bridge_rd = 1;
      bridge_state = 6;
      break;
    case 6: // Latch length
      ds_read_length = dut->bridge_rd_data;
      ds_read_cntr = 0;
      bridge_state = 7;
      printf("data-slot-read: id=%d, offset=%d, bridge_addr=0x%x, length=%d\n",
             ds_read_slot_id, ds_read_slot_offset, ds_read_bridge_address,
             ds_read_length);
      break;
    case 7: // Write data / status
      if (ds_read_cntr < ds_read_length) {
        dut->bridge_addr = ds_read_bridge_address + ds_read_cntr;
        dut->bridge_wr_data = 0;
        auto &fs = dataslots[ds_read_slot_id].first;
        for (unsigned i = 0; i < 4; i++) {
          uint8_t byte;
          fs.seekg(ds_read_slot_offset + ds_read_cntr + i, std::ios::beg);
          fs.read(reinterpret_cast<char *>(&byte), 1);
          dut->bridge_wr_data |= static_cast<uint32_t>(byte) << (8 * (3 - i));
        }
        dut->bridge_wr = 1;
        ds_read_cntr += 4;
      } else {
        dut->bridge_addr = 0xf8001000;
        dut->bridge_wr = 1;
        dut->bridge_wr_data = 0x6f6b0000;
        bridge_state = 2;
      }
      break;
    default:
      break;
    }
  }

  void RegisterDataSlot(uint16_t id, const std::string &path) {
    std::ifstream instream(path, std::ios::in | std::ios::binary);
    if (!instream) {
      std::cerr << "Unable to open '" << path << "'\n";
      exit(1);
    }
    instream.seekg(0, std::ios::end);
    auto size = instream.tellg();
    dataslots[id] = std::make_pair(std::move(instream), size);
    if (id < 16) { // Only lower 16 are mapped to update register
      updated_dataslots.push_back(id);
    }
  }
  void Finalize() { updated_dataslots_iter = updated_dataslots.begin(); }

private:
  int bridge_state = 0;
  uint32_t ds_read_slot_id;
  uint32_t ds_read_slot_offset;
  uint32_t ds_read_bridge_address;
  uint32_t ds_read_length;
  uint32_t ds_read_cntr;

  unsigned cntr = 0;

  std::map<uint16_t, std::pair<std::ifstream, uint32_t>> dataslots;
  decltype(dataslots)::iterator ds_it;
  std::vector<uint16_t> updated_dataslots;
  decltype(updated_dataslots)::iterator updated_dataslots_iter;
};

double sc_time_stamp() { return 0; }

int main(int argc, char *argv[]) {
  uint32_t exit_frame = 0;
  bool dump_video = false;

  std::string prg_path;
  std::string g64_path;
  std::string crt_path;

  std::string trace_path;
  std::vector<std::string> trace_modules;
  uint32_t trace_begin_frame = 0;

  CLI::App app{"Verilator based MyC64-pocket simulator"};
  app.add_flag("--dump-video", dump_video, "Dump video output as .png");
  app.add_option("--exit-frame", exit_frame, "Exit frame");
  app.add_option("--trace", trace_path, ".fst trace output");
  app.add_option("--trace-begin-frame", trace_begin_frame,
                 "Start trace on given frame")
      ->needs("--trace");
  app.add_option("--trace-modules", trace_modules, "Specify modules to trace")
      ->needs("--trace");
  CLI11_PARSE(app, argc, argv);

  // Initialize Verilators variables
  Verilated::commandArgs(argc, argv);
  Verilated::traceEverOn(!trace_path.empty());

  dut = std::make_unique<Vcore_top>();

  //
  // Bridge mockup - always present
  //
  BridgeHandler bridge;

  bridge.RegisterDataSlot(100, "image.bin");

  bridge.Finalize();

  std::unique_ptr<SimpleSDRAM> sdram = std::make_unique<SimpleSDRAM>();
  std::unique_ptr<SimpleUART> uart =
      std::make_unique<SimpleUART>(dut->debug_uart_tx);

  std::unique_ptr<TraceRTL> trace_rtl;
  if (!trace_path.empty()) {
    trace_rtl = std::make_unique<TraceRTL>(trace_path, trace_modules,
                                           trace_begin_frame);
  }

  std::unique_ptr<FrameDumper> framedumper;
  if (dump_video) {
    framedumper = std::make_unique<FrameDumper>();
  }

  dut->reset_n = 0;
  dut->eval();

  unsigned reset_cntr = 0;
  while (!Verilated::gotFinish()) {
    if (reset_cntr++ > 320) {
      dut->reset_n = 1;
    }
    dut->clk_74a = !dut->clk_74a;
    dut->eval();
    if (dut->clk_74a) {
      // Handle mockup bridge
      bridge.Tick();
      // Handle mockup SDRAM
      sdram->Tick();
      // Handle UART
      uart->Tick();
      // Frame dumper
      if (framedumper)
        framedumper->Tick();
      // Frame index increment if vsync comes after all handlers
      if (dut->video_vs) {
        g_frame_idx++;
        if (exit_frame != 0 && exit_frame == g_frame_idx) {
          exit(0);
        }
      }
    }
    dut->eval();
    dut->eval();
    if (trace_rtl) {
      trace_rtl->Tick();
    }
    g_ticks++;
  }

  return 0;
}
