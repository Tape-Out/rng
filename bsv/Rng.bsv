package Rng;

// 二值噪声源（引脚上一拍一位）→ SP 800-90B 第 4.4 节两项持续健康测试 → von Neumann 去偏 → 32 位字。
// 截止值在 RngHealth（BH）里按清单的熵估计算好。使能后先跑满一个 1024 位的窗口（4.3 第 4 条），
// 这期间的样本不出；任何一项测试报错就停止出字、置标志，软件把 en 清零再置一才重跑（4.3 第 2、5 条）。

import RegIf::*;
import RngRegs::*;
import RngHealth::*;

typedef struct {
  Bit#(0) none;
} RngCfg;

interface RngPins;
  (* always_ready, always_enabled, prefix = "" *)
  method Action noise((* port = "noise_i" *) Bit#(1) b);
endinterface

interface RngIfc#(numeric type aw, numeric type dw, numeric type hTenths);
  interface RegIf#(aw, dw) regs;
  interface RngPins        pins;
  (* always_ready *) method Bool irq;
endinterface

module mkRng#(RngCfg cfg)(RngIfc#(aw, dw, hTenths))
    provisos (Mul#(TDiv#(dw, 8), 8, dw), Add#(_a, 8, aw), Add#(_b, 1, dw),
              Add#(_c, 16, dw), Add#(_d, 32, dw));

  RngRegsIfc#(aw, dw) r <- mkRngRegs;

  Integer rctC = rctCutoff(valueOf(hTenths));
  Integer aptC = aptCutoff(valueOf(hTenths));

  Wire#(Bit#(1)) noiseIn <- mkBypassWire;
  // 读走一个字的脉冲在总线方法之后才有，而引擎要在它之前读寄存器：隔一拍，由 CReg 递过去
  Reg#(Bool) take[2] <- mkCReg(2, False);

  Reg#(Bool)            prevEn <- mkReg(False);
  Reg#(Bool)            first  <- mkReg(True);
  Reg#(Bit#(1))         rA     <- mkReg(0);
  Reg#(UInt#(8))        rB     <- mkReg(0);
  Reg#(Bit#(1))         aA     <- mkReg(0);
  Reg#(UInt#(11))       aB     <- mkReg(0);
  Reg#(UInt#(11))       aI     <- mkReg(0);
  Reg#(Bool)            rctF   <- mkReg(False);
  Reg#(Bool)            aptF   <- mkReg(False);
  Reg#(Bool)            ready  <- mkReg(False);
  Reg#(Maybe#(Bit#(1))) half   <- mkReg(tagged Invalid);
  Reg#(Bit#(32))        word   <- mkReg(0);
  Reg#(UInt#(6))        nbits  <- mkReg(0);
  Reg#(Bool)            valid  <- mkReg(False);

  // 刚被读走的那一拍 valid 还没清，这里先盖掉，背靠背两次读不会拿到同一个字
  Bool offer = valid && !take[0];
  // mark 写 CReg 的 1 口，排在 step 之后，不能再读 step 写的 valid；这一拍摆没摆字由 show 递过去
  PulseWire offered <- mkPulseWire;

  // 读 data 只在那一拍确实有字摆着时才算取走：没字时读出 0，脉冲不许把下一拍才攒好的字丢掉
  rule mark;
    if (r.data_rd && offered) take[1] <= True;
  endrule

  rule step;
    Bit#(1) x    = noiseIn;
    Bool    en   = r.ctrl_en == 1;
    Bool    rise = en && !prevEn;

    Bool            nFirst = first;
    Bit#(1)         nRA    = rA;
    UInt#(8)        nRB    = rB;
    Bit#(1)         nAA    = aA;
    UInt#(11)       nAB    = aB;
    UInt#(11)       nAI    = aI;
    Bool            nRctF  = rctF;
    Bool            nAptF  = aptF;
    Bool            nReady = ready;
    Maybe#(Bit#(1)) nHalf  = half;
    Bit#(32)        nWord  = word;
    UInt#(6)        nBits  = nbits;
    Bool            nValid = valid;

    if (take[0] && valid) begin nValid = False; nBits = 0; end

    if (rise) begin
      nFirst = True; nAI = 0; nRctF = False; nAptF = False; nReady = False;
      nHalf = tagged Invalid; nBits = 0; nValid = False;
    end else if (en) begin
      // 重复计数测试（4.4.1）：计数饱和在截止值，不回绕
      if (first) begin
        nRA = x; nRB = 1; nFirst = False;
      end else if (x == rA) begin
        if (rB < fromInteger(rctC)) nRB = rB + 1;
        if (rB + 1 >= fromInteger(rctC)) nRctF = True;
      end else begin
        nRA = x; nRB = 1;
      end

      // 自适应比例测试（4.4.2），窗口 1024
      if (aI == 0) begin
        nAA = x; nAB = 1; nAI = 1;
      end else begin
        UInt#(11) b = aB + ((x == aA) ? 1 : 0);
        if (b >= fromInteger(aptC)) nAptF = True;
        nAB = b;
        if (aI == 1023) begin
          // 二值源的扩展：补集个数也不许到截止值
          if (1024 - b >= fromInteger(aptC)) nAptF = True;
          nAI = 0;
          if (!rctF && !aptF && !nRctF && !nAptF) nReady = True;
        end else
          nAI = aI + 1;
      end

      // 去偏与出字：开机测试过了、没有报错、手上没有等着取的字才收
      if (ready && !nRctF && !nAptF && !nValid) begin
        if (half matches tagged Valid .h) begin
          if (h != x) begin
            nWord = {word[30:0], h};
            if (nbits == 31) begin nValid = True; nBits = 0; end
            else nBits = nbits + 1;
          end
          nHalf = tagged Invalid;
        end else
          nHalf = tagged Valid x;
      end
      // 报错就不出字，已经攒好的那一个也不交（4.3 第 2 条）
      if (nRctF || nAptF) nValid = False;
    end

    prevEn <= en;
    take[0] <= False;
    first <= nFirst; rA <= nRA; rB <= nRB;
    aA <= nAA; aB <= nAB; aI <= nAI;
    rctF <= nRctF; aptF <= nAptF; ready <= nReady;
    half <= nHalf; word <= nWord; nbits <= nBits; valid <= nValid;
  endrule

  rule show;
    if (offer) offered.send;
    r.status_valid_in(offer ? 1 : 0);
    r.status_ready_in(ready ? 1 : 0);
    r.status_rctfail_in(rctF ? 1 : 0);
    r.status_aptfail_in(aptF ? 1 : 0);
    r.data_in(offer ? word : 0);
    r.cutoff_rct_in(fromInteger(rctC));
    r.cutoff_apt_in(fromInteger(aptC));
  endrule

  interface regs = r.regs;
  interface RngPins pins;
    method Action noise(Bit#(1) b);
      noiseIn <= b;
    endmethod
  endinterface
  method Bool irq = r.ctrl_ien == 1 && (offer || rctF || aptF);
endmodule

endpackage
