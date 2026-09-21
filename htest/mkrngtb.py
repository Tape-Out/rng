"""rng 的行为测试台：截止值自己算，噪声源用几种模型驱动。

截止值不照被测件算：RCT 用分数算 C = 1 + ⌈20 / H⌉；APT 取 SP 800-90B 表 2，H = 1 那一格另用组合数精确算临界值自证。
噪声源模型：交替的 0、1 · 32 位 LFSR（脚本先把要喂的 16384 位对两项测试查一遍，保证真源不会误报）·
卡死在 0 · 十五个 1 接一个 0。判据见 notes/规范对照/rng.md。
"""
import json
import pathlib
import sys
from fractions import Fraction
from math import ceil, comb

out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ".")
out.mkdir(parents=True, exist_ok=True)
cfg = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
label = cfg.get("label", "")
h = int(cfg.get("knobs", {}).get("hTenths", 4))

TABLE2 = {2: 941, 4: 840, 6: 748, 8: 664, 10: 589}   # 表 2，二值源，W = 1024
if h not in TABLE2:
    raise SystemExit(f"hTenths={h} 不在表 2 里，不生成测试台")
RCT = 1 + ceil(Fraction(20) / Fraction(h, 10))
APT = TABLE2[h]


def exact_half(w):
    alpha = Fraction(1, 2**20)
    total = 0
    for k in range(w + 1):
        total += comb(w, k)
        if Fraction(total, 2**w) >= 1 - alpha:
            return k + 1


if exact_half(1024) != TABLE2[10]:
    raise SystemExit("表 2 的 H = 1 与精确算出的二项分布临界值对不上，不生成测试台")

TAPS = 0x80200003
N = 16384


def lfsr_bits(seed, n):
    s, bits = seed, []
    for _ in range(n):
        bits.append(s & 1)
        s = (s >> 1) ^ TAPS if s & 1 else s >> 1
    return bits


def safe(bits):
    run, longest = 1, 1
    for a, b in zip(bits, bits[1:]):
        run = run + 1 if a == b else 1
        longest = max(longest, run)
    if longest >= RCT - 2:
        return False
    ones = sum(bits[:1024])
    for s in range(len(bits) - 1024):
        if ones >= APT - 8 or 1024 - ones >= APT - 8:
            return False
        ones += bits[s + 1024] - bits[s]
    return True


seed = next(s for s in range(0x1234567, 0x1234567 + 64) if safe(lfsr_bits(s, N)))

verdict = (f"cutoffs rct {RCT} and apt {APT} match SP 800-90B, words wait for a 1024-sample startup, an alternating "
           f"source debiases to all-zero or all-one words, a pseudo-random source gives distinct words, a word is never "
           f"delivered twice, polling data every cycle loses no word, a stuck source trips only the repetition count test and stops output, fifteen ones and a "
           f"zero trip only the adaptive proportion test, enabling again clears the flags, and irq follows ien")

TEMPLATE = r'''package Rng@L@Tb;

// 由 htest/mkrngtb.py 生成，勿手改。这一点：hTenths=@H@，截止值 rct @RCT@、apt @APT@，LFSR 种子 @SEED@

import StmtFSM::*;
import ConfigReg::*;
import RegIf::*;
import Rng::*;

(* synthesize *)
module mkRng@L@Tb(Empty);
  RngIfc#(8, 32, @H@) d <- mkRng(RngCfg { none: ? });

  // drive 写、命令序列读：用 ConfigReg，两边才排得出先后（否则 G0010）
  Reg#(UInt#(32)) cyc     <- mkConfigReg(0);
  Reg#(Bit#(2))   mode    <- mkReg(0);   // 0 交替 · 1 伪随机 · 2 卡死在 0 · 3 十五个 1 接一个 0
  Reg#(Bit#(1))   alt     <- mkReg(0);
  Reg#(Bit#(32))  lfsr[2] <- mkCReg(2, 32'h@SEED@);
  Reg#(UInt#(4))  ph16    <- mkReg(0);

  Bit#(1) noiseBit = mode == 0 ? alt : (mode == 1 ? lfsr[0][0] : (mode == 2 ? 0 : (ph16 == 15 ? 0 : 1)));

  rule drive;
    d.pins.noise(noiseBit);
    alt <= ~alt;
    lfsr[0] <= (lfsr[0][0] == 1) ? ((lfsr[0] >> 1) ^ 32'h@TAPS@) : (lfsr[0] >> 1);
    ph16 <= ph16 + 1;
    cyc <= cyc + 1;
    if (cyc > @LIMIT@) begin
      $display("TIMEOUT");
      $finish(1);
    end
  endrule

  Reg#(Bool)      bad       <- mkReg(False);
  Reg#(Bit#(32))  rd        <- mkReg(0);
  Reg#(UInt#(32)) t0        <- mkReg(0);
  Reg#(Bool)      seenValid <- mkReg(False);
  Reg#(UInt#(8))  n         <- mkReg(0);
  Reg#(Bit#(32))  w0        <- mkReg(0);
  Reg#(Bit#(32))  w1        <- mkReg(0);
  Reg#(Bit#(32))  w2        <- mkReg(0);
  Reg#(Bit#(32))  w3        <- mkReg(0);

  function Action rdReg(Bit#(8) a) = action
    let x <- d.regs.access(RegReq { addr: a, write: False, wdata: 0, wstrb: 4'hF });
    rd <= x.rdata;
  endaction;

  function Action wrReg(Bit#(8) a, Bit#(32) v) = action
    let x <- d.regs.access(RegReq { addr: a, write: True, wdata: v, wstrb: 4'hF });
  endaction;

  function Stmt waitValid(String what) = seq
    action t0 <= cyc; endaction
    rdReg(8'h04);
    while (rd[0] == 0 && cyc - t0 < 4000) rdReg(8'h04);
    action
      if (rd[0] == 0) begin
        $display("FAIL no word came within 4000 cycles %s (status %02h)", what, rd[3:0]);
        bad <= True;
      end
    endaction
  endseq;

  Stmt test = seq
    rdReg(8'h0C);
    action
      if (rd[15:0] != @RCT@ || rd[31:16] != @APT@) begin
        $display("FAIL cutoff reads rct %0d apt %0d, want @RCT@ and @APT@", rd[15:0], rd[31:16]);
        bad <= True;
      end
    endaction

    // 开机：交替源，使能后至少 1024 拍才 ready，这之间不出字
    action mode <= 0; seenValid <= False; endaction
    action wrReg(8'h00, 1); t0 <= cyc; endaction
    rdReg(8'h04);
    while (rd[1] == 0 && cyc - t0 < 3000) action
      let x <- d.regs.access(RegReq { addr: 8'h04, write: False, wdata: 0, wstrb: 4'hF });
      rd <= x.rdata;
      if (x.rdata[0] == 1) seenValid <= True;
    endaction
    action
      Bool wrong = False;
      if (rd[1] == 0) begin
        $display("FAIL ready never rose after enabling"); wrong = True;
      end else if (cyc - t0 < 1024) begin
        $display("FAIL ready rose %0d cycles after enabling, before a 1024-sample startup window", cyc - t0); wrong = True;
      end
      if (seenValid) begin $display("FAIL a word was offered before the startup tests passed"); wrong = True; end
      if (wrong) bad <= True;
    endaction

    // 交替源：去偏后全 0 或全 1；读走之后紧接着再读，不许拿到同一个字
    n <= 0;
    while (n < 3) seq
      waitValid("from the alternating source");
      rdReg(8'h08);
      action
        let x <- d.regs.access(RegReq { addr: 8'h08, write: False, wdata: 0, wstrb: 4'hF });
        Bool wrong = False;
        if (rd != 0 && rd != 32'hFFFFFFFF) begin
          $display("FAIL an alternating source gave %08h, want all zeros or all ones", rd); wrong = True;
        end
        if (x.rdata != 0) begin
          $display("FAIL reading data again right away gave %08h, the word was delivered twice", x.rdata); wrong = True;
        end
        if (wrong) bad <= True;
      endaction
      n <= n + 1;
    endseq

    // 伪随机源：四个字两两不同
    action mode <= 1; lfsr[1] <= 32'h@SEED@; endaction
    waitValid("from the pseudo-random source");
    rdReg(8'h08);
    action w0 <= rd; endaction
    waitValid("from the pseudo-random source");
    rdReg(8'h08);
    action w1 <= rd; endaction
    waitValid("from the pseudo-random source");
    rdReg(8'h08);
    action w2 <= rd; endaction
    waitValid("from the pseudo-random source");
    rdReg(8'h08);
    action w3 <= rd; endaction
    rdReg(8'h04);
    action
      Bool wrong = False;
      if (w0 == w1 || w0 == w2 || w0 == w3 || w1 == w2 || w1 == w3 || w2 == w3) begin
        $display("FAIL the pseudo-random source repeated a word: %08h %08h %08h %08h", w0, w1, w2, w3); wrong = True;
      end
      if (rd[3:2] != 0) begin
        $display("FAIL a health test tripped on the pseudo-random source (status %02h)", rd[3:0]); wrong = True;
      end
      if (wrong) bad <= True;
    endaction

    // 不看状态、每拍都读 data：没字时的读不许算取走，否则每个字一攒好就被前一拍的空读丢掉
    action t0 <= cyc; n <= 0; w0 <= 0; endaction
    while (cyc - t0 < 1500) action
      let x <- d.regs.access(RegReq { addr: 8'h08, write: False, wdata: 0, wstrb: 4'hF });
      if (x.rdata != 0) begin
        if (x.rdata == w0) begin $display("FAIL polling data every cycle gave %08h twice in a row", x.rdata); bad <= True; end
        w0 <= x.rdata;
        if (n < 255) n <= n + 1;
      end
    endaction
    action
      if (n < 3) begin $display("FAIL polling data every cycle for 1500 cycles got %0d words, want at least 3", n); bad <= True; end
    endaction

    // 中断跟着 ien
    waitValid("before the interrupt check");
    action if (d.irq) begin $display("FAIL irq is high with a word waiting but ien off"); bad <= True; end endaction
    wrReg(8'h00, 3);
    action if (!d.irq) begin $display("FAIL irq stays low with a word waiting and ien on"); bad <= True; end endaction
    wrReg(8'h00, 1);

    // 卡死在 0：只该是重复计数测试报，报了之后不再出字
    action mode <= 2; t0 <= cyc; endaction
    rdReg(8'h04);
    while (rd[2] == 0 && cyc - t0 < 400) rdReg(8'h04);
    action
      Bool wrong = False;
      if (rd[2] == 0) begin
        $display("FAIL a source stuck at 0 did not trip the repetition count test within 400 cycles"); wrong = True;
      end else if (cyc - t0 > @RCT@ + 40) begin
        $display("FAIL the repetition count test tripped %0d cycles after the source stuck, cutoff @RCT@", cyc - t0); wrong = True;
      end
      if (rd[3] == 1) begin
        $display("FAIL the adaptive proportion test tripped before the repetition count test on a stuck source"); wrong = True;
      end
      if (wrong) bad <= True;
    endaction
    action t0 <= cyc; seenValid <= False; endaction
    while (cyc - t0 < 1500) action
      let x <- d.regs.access(RegReq { addr: 8'h04, write: False, wdata: 0, wstrb: 4'hF });
      if (x.rdata[0] == 1) seenValid <= True;
    endaction
    action if (seenValid) begin $display("FAIL a word was offered after the repetition count test failed"); bad <= True; end endaction

    // 清掉 en 再置一：标志清零，伪随机源又出字
    action mode <= 1; lfsr[1] <= 32'h@SEED@; endaction
    wrReg(8'h00, 0);
    wrReg(8'h00, 1);
    delay(4);
    rdReg(8'h04);
    action
      if (rd[3:2] != 0) begin $display("FAIL the failure flags stayed set after enabling again (status %02h)", rd[3:0]); bad <= True; end
    endaction
    waitValid("after enabling again");

    // 十五个 1 接一个 0：只该是自适应比例测试报
    action mode <= 3; t0 <= cyc; endaction
    rdReg(8'h04);
    while (rd[3] == 0 && cyc - t0 < 3000) rdReg(8'h04);
    action
      Bool wrong = False;
      if (rd[3] == 0) begin
        $display("FAIL fifteen ones and a zero did not trip the adaptive proportion test within 3000 cycles"); wrong = True;
      end
      if (rd[2] == 1) begin
        $display("FAIL the repetition count test tripped on runs of fifteen"); wrong = True;
      end
      if (wrong) bad <= True;
    endaction
  endseq;

  FSM fsm <- mkFSM(test);
  Reg#(Bool) started <- mkReg(False);

  rule go (!started);
    started <= True;
    fsm.start;
  endrule

  rule fin (started && fsm.done);
    if (bad) $display("FAILED");
    else $display("PASS rng: @VERDICT@");
    $finish(bad ? 1 : 0);
  endrule
endmodule

endpackage
'''

txt = (TEMPLATE.replace("@L@", label).replace("@H@", str(h))
       .replace("@RCT@", str(RCT)).replace("@APT@", str(APT))
       .replace("@SEED@", f"{seed:08X}").replace("@TAPS@", f"{TAPS:08X}")
       .replace("@LIMIT@", "60000").replace("@VERDICT@", verdict))

(out / f"Rng{label}Tb.bsv").write_text(txt, encoding="utf-8")
print(f"  rng 行为测试台就位：hTenths={h} rct={RCT} apt={APT} seed={seed:#x}")
