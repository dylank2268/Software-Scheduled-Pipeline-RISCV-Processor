--Dylan Kramer and Michael Berg
--Top level implementation of a single-cycle RISC-V processor
library IEEE;
use IEEE.std_logic_1164.all;
library work;
use ieee.numeric_std.all;
use work.RISCV_types.all;

entity RISCV_Processor is
  generic(N : integer := DATA_WIDTH);
  port(iCLK            : in std_logic;
       iRST            : in std_logic;
       iInstLd         : in std_logic;
       iInstAddr       : in std_logic_vector(N-1 downto 0);
       iInstExt        : in std_logic_vector(N-1 downto 0);
       oALUOut         : out std_logic_vector(N-1 downto 0)); -- TODO: Hook this up to the output of the ALU. It is important for synthesis that you have this output that can effectively be impacted by all other components so they are not optimized away.

end  RISCV_Processor;


architecture structure of RISCV_Processor is

  -- Required data memory signals
  signal s_DMemWr       : std_logic; -- TODO: use this signal as the final active high data memory write enable signal
  signal s_DMemAddr     : std_logic_vector(N-1 downto 0); -- TODO: use this signal as the final data memory address input
  signal s_DMemData     : std_logic_vector(N-1 downto 0); -- TODO: use this signal as the final data memory data input
  signal s_DMemOut      : std_logic_vector(N-1 downto 0); -- TODO: use this signal as the data memory output
 
  -- Required register file signals 
  signal s_RegWr        : std_logic; -- TODO: use this signal as the final active high write enable input to the register file
  signal s_RegWrAddr    : std_logic_vector(4 downto 0); -- TODO: use this signal as the final destination register address input
  signal s_RegWrData    : std_logic_vector(N-1 downto 0); -- TODO: use this signal as the final data memory data input

  -- Required instruction memory signals
  signal s_IMemAddr     : std_logic_vector(N-1 downto 0); -- Do not assign this signal, assign to s_NextInstAddr instead
  signal s_NextInstAddr : std_logic_vector(N-1 downto 0); -- TODO: use this signal as your intended final instruction memory address input.
  signal s_Inst         : std_logic_vector(N-1 downto 0) := (others=> '0'); -- TODO: use this signal as the instruction signal 

  -- Required halt signal -- for simulation
  signal s_Halt         : std_logic;  -- TODO: this signal indicates to the simulation that intended program execution has completed. (Opcode: 01 0100)

  -- Required overflow signal -- for overflow exception detection
  signal s_Ovfl         : std_logic;  -- TODO: this signal indicates an overflow exception would have been initiated

  component mem is
    generic(ADDR_WIDTH : integer;
            DATA_WIDTH : integer);
    port(
          clk          : in std_logic;
          addr         : in std_logic_vector((ADDR_WIDTH-1) downto 0);
          data         : in std_logic_vector((DATA_WIDTH-1) downto 0);
          we           : in std_logic := '1';
          q            : out std_logic_vector((DATA_WIDTH -1) downto 0));
    end component;

  -- TODO: You may add any additional signals or components your implementation 
  --       requires below this comment

--PC Path signals
signal s_PC : std_logic_vector(N-1 downto 0) := x"00000000";
signal s_PCPlus4 : std_logic_vector(N-1 downto 0);
signal PCSrc : pc_src_t;


--Decode fields
signal s_opcode  : std_logic_vector(6 downto 0);
signal s_funct3  : std_logic_vector(2 downto 0);
signal s_funct7  : std_logic_vector(6 downto 0);
signal s_rs1     : std_logic_vector(4 downto 0);
signal s_rs2     : std_logic_vector(4 downto 0);
signal s_rd      : std_logic_vector(4 downto 0);

--Register signals
signal s_rs1_val : std_logic_vector(N-1 downto 0) := (others=>'0');
signal s_rs2_val : std_logic_vector(N-1 downto 0) := (others=>'0');

--Immediate signals
signal s_ImmKind : std_logic_vector(2 downto 0); --Selects what instruction type for control unit
signal s_immI : std_logic_vector(N-1 downto 0) := (others => '0');
signal s_immB : std_logic_vector(31 downto 0) := (others => '0');
signal s_immJ : std_logic_vector(31 downto 0) := (others => '0');

--ALU signals
signal s_ALUSrcSel : std_logic := '0'; --0: rs2, 1: immI
signal s_ALUInB : std_logic_vector(N-1 downto 0); --second ALU input
signal s_ALURes : std_logic_vector(N-1 downto 0); --ALU Result signal
signal s_ALUCtrl : std_logic_vector(3 downto 0) := (others=>'0');
signal s_ALUOvfl : std_logic;
signal s_ALU2BitControl : std_logic_vector(1 downto 0);
signal s_ALUShiftAmt : std_logic_vector(4 downto 0);
signal s_ALUZero : std_logic := '0'; --Zero flag signal

--Writeback signals
signal s_WBSel : std_logic_vector(1 downto 0) := "00";  -- 00=ALU, 01=Load, 10=PC+4, 11=unused
signal s_WBData : std_logic_vector(31 downto 0);

--Load/Store control signals
signal s_MemRead : std_logic;
signal s_MemWrite : std_logic;
signal s_LdByte : std_logic;
signal s_LdHalf : std_logic;
signal s_LdUnsigned : std_logic;


--Load/Store unit signals
signal s_RegWrLoad : std_logic;
signal s_RegWr_Final : std_logic;

--Signals for AUIPC logic
signal s_ALUInA : std_logic_vector(31 downto 0);
signal s_ASel   : std_logic_vector(1 downto 0); -- 00=RS1, 01=PC, 10=ZERO

--Signals for branch logic
signal s_Branch : std_logic; --From control unit
signal s_BranchTaken : std_logic; --From branch logic 

--Instruction fetch stage wires
signal s_PCF : std_logic_vector(N-1 downto 0);
signal s_PCPlus4F : std_logic_vector(N-1 downto 0);
signal s_InstrF : std_logic_vector(N-1 downto 0);
--Instruction fetch/Decode outputs
signal s_PCD : std_logic_vector(N-1 downto 0);
signal s_PCPlus4D : std_logic_vector(N-1 downto 0);
signal s_InstrD : std_logic_vector(N-1 downto 0);


--Decode stage signals
signal s_RS1_Val_D   : std_logic_vector(31 downto 0);
signal s_RS2_Val_D   : std_logic_vector(31 downto 0);
signal s_Imm_D       : std_logic_vector(31 downto 0);
signal s_Halt_D      : std_logic;
signal s_LdUnsigned_D: std_logic;
signal s_LdHalf_D    : std_logic;
signal s_LdByte_D    : std_logic;
signal s_MemWrite_D  : std_logic;
signal s_MemRead_D   : std_logic;
signal s_WBSel_D     : std_logic_vector(1 downto 0);
signal s_Branch_D    : std_logic;
signal s_ImmKind_D   : std_logic_vector(2 downto 0);
signal s_RegWr_D     : std_logic;
signal s_ALUSrcSel_D : std_logic;
signal s_AluCtrl_D   : std_logic_vector(3 downto 0);
signal s_Funct3_D    : std_logic_vector(2 downto 0);
signal s_ASel_D      : std_logic_vector(1 downto 0);

--Execute stage signals
signal s_PC_E        : std_logic_vector(31 downto 0);
signal s_PCPlus4_E   : std_logic_vector(31 downto 0);
signal s_RS1_Val_E   : std_logic_vector(31 downto 0);
signal s_RS2_Val_E   : std_logic_vector(31 downto 0);
signal s_Imm_E       : std_logic_vector(31 downto 0);
signal s_RS1_E       : std_logic_vector(4 downto 0);
signal s_RS2_E       : std_logic_vector(4 downto 0);
signal s_RD_E        : std_logic_vector(4 downto 0);
signal s_Halt_E      : std_logic;
signal s_LdUnsigned_E: std_logic;
signal s_LdHalf_E    : std_logic;
signal s_LdByte_E    : std_logic;
signal s_MemWrite_E  : std_logic;
signal s_MemRead_E   : std_logic;
signal s_WBSel_E     : std_logic_vector(1 downto 0);
signal s_Branch_E    : std_logic;
signal s_ImmKind_E   : std_logic_vector(2 downto 0);
signal s_RegWr_E     : std_logic;
signal s_ALUSrcSel_E : std_logic;
signal s_AluCtrl_E   : std_logic_vector(3 downto 0);
signal s_Funct3_E    : std_logic_vector(2 downto 0);
signal s_ASel_E      : std_logic_vector(1 downto 0);



--Memory signals
signal s_PCPlus4_M    : std_logic_vector(31 downto 0);
signal s_ALURes_M     : std_logic_vector(31 downto 0);
signal s_WriteData_M  : std_logic_vector(31 downto 0);
signal s_RD_M         : std_logic_vector(4 downto 0);

signal s_RegWr_M      : std_logic;
signal s_MemRead_M    : std_logic;
signal s_MemWrite_M   : std_logic;
signal s_WBSel_M      : std_logic_vector(1 downto 0);
signal s_LdByte_M     : std_logic;
signal s_LdHalf_M     : std_logic;
signal s_LdUnsigned_M : std_logic;
signal s_Halt_M      : std_logic;

--Signals for MEM/WB
signal s_LoadData_M    : std_logic_vector(31 downto 0);
signal s_PCPlus4_W   : std_logic_vector(31 downto 0);
signal s_ALURes_W    : std_logic_vector(31 downto 0);
signal s_LoadData_W  : std_logic_vector(31 downto 0);
signal s_RD_W        : std_logic_vector(4 downto 0);
signal s_RegWr_W     : std_logic;
signal s_WBSel_W     : std_logic_vector(1 downto 0);
signal s_Halt_W      : std_logic;
-- Final writeback bus
signal s_WBData_W    : std_logic_vector(31 downto 0);




--Control unit instantiation
  component ControlUnit is
    port(
      opcode     : in  std_logic_vector(6 downto 0);
      funct3     : in  std_logic_vector(2 downto 0);
      funct7     : in  std_logic_vector(6 downto 0);
      imm12      : in  std_logic_vector(11 downto 0);
      ALUSrc     : out std_logic;
      ALUControl : out std_logic_vector(1 downto 0);
      ImmType    : out std_logic_vector(2 downto 0);
      ResultSrc  : out std_logic_vector(1 downto 0);
      MemWrite   : out std_logic;
      RegWrite   : out std_logic;
      ALU_op     : out std_logic_vector(3 downto 0);
      Halt       : out std_logic;
      MemRead    : out std_logic;
      LdByte     : out std_logic;
      LdHalf     : out std_logic;
      LdUnsigned : out std_logic;
      ASel       : out std_logic_vector(1 downto 0);
      Branch        : out std_logic;
      PCSrc         : out pc_src_t
    );
end component;
--N carry ripple full adder instantiation
  component n_ripple_full_adder is
    generic(N: integer := 8);
    port(
      D0   : in  std_logic_vector(N-1 downto 0);
      D1   : in  std_logic_vector(N-1 downto 0);
      Cin  : in  std_logic;
      S    : out std_logic_vector(N-1 downto 0);
      Cout : out std_logic
    );
end component;


--ALU unit instantiation
  component ALUUnit is
    port (
      A         : in  std_logic_vector(31 downto 0);
      B         : in  std_logic_vector(31 downto 0);
      shift_amt : in  std_logic_vector(4 downto 0);
      ALU_op    : in  std_logic_vector(3 downto 0);  -- matches your fixed encodings
      F         : out std_logic_vector(31 downto 0);
      Zero      : out std_logic;
      Overflow  : out std_logic
    );
end component;


--reg file instantiation
  component reg is
    generic(N : integer := DATA_WIDTH);
    port(
      RS1     : in  std_logic_vector(4 downto 0);
      RS2     : in  std_logic_vector(4 downto 0);
      DATA_IN : in  std_logic_vector(N-1 downto 0);
      W_SEL   : in  std_logic_vector(4 downto 0);
      WE      : in  std_logic;
      RST     : in  std_logic;
      CLK     : in  std_logic;
      RS1_OUT : out std_logic_vector(N-1 downto 0);
      RS2_OUT : out std_logic_vector(N-1 downto 0)
    );
end component;
--N-bit 2t1 mux instantiation
  component mux2t1_N is
    generic(N : integer := 32);
    port(
      i_S  : in  std_logic;
      i_D0 : in  std_logic_vector(N-1 downto 0);
      i_D1 : in  std_logic_vector(N-1 downto 0);
      o_O  : out std_logic_vector(N-1 downto 0)
    );
end component;
--Immediate generator instantiation
  component imm_generator is
    port(
      i_instr : in  std_logic_vector(31 downto 0);
      i_kind  : in  std_logic_vector(2 downto 0);  -- 000=R,001=I,010=S,011=SB,100=U,101=UJ
      o_imm   : out std_logic_vector(31 downto 0)
    );
end component;
--PC Fetch component instantiation
  component PCFetch is
    generic (G_RESET_VECTOR : unsigned(31 downto 0) := x"00000000");
    port (
      i_clk       : in  std_logic;
      i_rst       : in  std_logic;
      i_halt      : in  std_logic;
      i_pc_src    : in  pc_src_t;    -- SEQ, BR_TGT, JAL_TGT, JALR_TGT
      i_br_taken  : in  std_logic;
      i_rs1_val   : in  std_logic_vector(31 downto 0); -- for JALR
      i_immI      : in  std_logic_vector(31 downto 0);
      i_immB      : in  std_logic_vector(31 downto 0);
      i_immJ      : in  std_logic_vector(31 downto 0);
      o_pc        : out std_logic_vector(31 downto 0);
      o_pc_plus4  : out std_logic_vector(31 downto 0);
      o_imem_addr : out std_logic_vector(31 downto 0)
    );
  end component;

  --Load and store unit instantiation
component load_store_unit is
    port (
      i_addr        : in  std_logic_vector(31 downto 0);
      i_rs2_wdata   : in  std_logic_vector(31 downto 0);
      i_mem_read    : in  std_logic;
      i_mem_write   : in  std_logic;
      i_ld_byte     : in  std_logic;
      i_ld_half     : in  std_logic;
      i_ld_unsigned : in  std_logic;
      i_mem_rdata   : in  std_logic_vector(31 downto 0);
      o_mem_addr    : out std_logic_vector(31 downto 0);
      o_mem_wdata   : out std_logic_vector(31 downto 0);
      o_mem_we      : out std_logic;
      o_load_data   : out std_logic_vector(31 downto 0)
    );
  end component;

  --Branch logic unit
  component branch_logic is 
    port(
    i_rs1     : in  std_logic_vector(31 downto 0);
    i_rs2     : in  std_logic_vector(31 downto 0);
    i_funct3  : in  std_logic_vector(2 downto 0);
    i_branch  : in  std_logic;  -- from control unit (1 for branch instructions)
    o_br_taken: out std_logic
    );
  end component;
--4t1 mux instantiation
component mux4t1_N is
  generic (N : integer := 32);
  port(
    i_S  : in  std_logic_vector(1 downto 0);
    i_D0 : in  std_logic_vector(N-1 downto 0);
    i_D1 : in  std_logic_vector(N-1 downto 0);
    i_D2 : in  std_logic_vector(N-1 downto 0);
    i_D3 : in  std_logic_vector(N-1 downto 0);
    o_O  : out std_logic_vector(N-1 downto 0)
  );
end component;

--IF/ID Reg Instantiation
component IF_ID_Reg is
  port(
    i_CLK         : in  std_logic;
    i_RST         : in  std_logic;
    i_WE          : in  std_logic;
    i_PC          : in  std_logic_vector(31 downto 0);
    i_PC_Plus4    : in  std_logic_vector(31 downto 0);
    i_Instruction : in  std_logic_vector(31 downto 0);
    o_PC          : out std_logic_vector(31 downto 0);
    o_PC_Plus4    : out std_logic_vector(31 downto 0);
    o_Instruction : out std_logic_vector(31 downto 0)
  );
end component;

--ID/EX Reg instantiation
component ID_EX_Reg is
  generic(DATA_WIDTH : integer := 32);
  port(
    CLK           : in  std_logic;
    RST           : in  std_logic;
    EN            : in  std_logic;
    PCD           : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    PCPLUS4D      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    RD1D          : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    RD2D          : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    IMMD          : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    RS1D          : in  std_logic_vector(4 downto 0);
    RS2D          : in  std_logic_vector(4 downto 0);
    RDD           : in  std_logic_vector(4 downto 0);

    -- NEW: pipe Funct3 and ASel
    Funct3_D      : in  std_logic_vector(2 downto 0);
    ASel_D        : in  std_logic_vector(1 downto 0);
    Halt_D        : in  std_logic;
    LdUnsigned_D  : in  std_logic;
    LdHalf_D      : in  std_logic;
    LdByte_D      : in  std_logic;
    MemWrite_D    : in  std_logic;
    MemRead_D     : in  std_logic;
    WBSel_D       : in  std_logic_vector(1 downto 0);
    Branch_D      : in  std_logic;
    ImmKind_D     : in  std_logic_vector(2 downto 0);
    RegWr_D       : in  std_logic;
    ALUSrcSel_D   : in  std_logic;
    AluCtrl_D     : in  std_logic_vector(3 downto 0);
    PCE           : out std_logic_vector(DATA_WIDTH-1 downto 0);
    PCPLUS4E      : out std_logic_vector(DATA_WIDTH-1 downto 0);
    RD1E          : out std_logic_vector(DATA_WIDTH-1 downto 0);
    RD2E          : out std_logic_vector(DATA_WIDTH-1 downto 0);
    IMME          : out std_logic_vector(DATA_WIDTH-1 downto 0);
    RS1E          : out std_logic_vector(4 downto 0);
    RS2E          : out std_logic_vector(4 downto 0);
    RDE           : out std_logic_vector(4 downto 0);
    Funct3_E      : out std_logic_vector(2 downto 0);
    ASel_E        : out std_logic_vector(1 downto 0);
    Halt_E        : out std_logic;
    LdUnsigned_E  : out std_logic;
    LdHalf_E      : out std_logic;
    LdByte_E      : out std_logic;
    MemWrite_E    : out std_logic;
    MemRead_E     : out std_logic;
    WBSel_E       : out std_logic_vector(1 downto 0);
    Branch_E      : out std_logic;
    ImmKind_E     : out std_logic_vector(2 downto 0);
    RegWr_E       : out std_logic;
    ALUSrcSel_E   : out std_logic;
    AluCtrl_E     : out std_logic_vector(3 downto 0)
  );
end component;

component EX_MEM_reg is
  port(
    i_CLK         : in  std_logic;
    i_RST         : in  std_logic;
    i_WE          : in  std_logic;
    i_Halt        : in std_logic;
    i_PC_Plus4    : in  std_logic_vector(31 downto 0);
    i_ALU_Result  : in  std_logic_vector(31 downto 0);
    i_WriteData   : in  std_logic_vector(31 downto 0);
    i_RD          : in  std_logic_vector(4 downto 0);
    i_RegWrite    : in  std_logic;
    i_MemRead     : in  std_logic;
    i_MemWrite    : in  std_logic;
    i_WBSel       : in  std_logic_vector(1 downto 0);
    i_LdByte      : in  std_logic;
    i_LdHalf      : in  std_logic;
    i_LdUnsigned  : in  std_logic;
    o_PC_Plus4    : out std_logic_vector(31 downto 0);
    o_ALU_Result  : out std_logic_vector(31 downto 0);
    o_WriteData   : out std_logic_vector(31 downto 0);
    o_RD          : out std_logic_vector(4 downto 0);
    o_RegWrite    : out std_logic;
    o_MemRead     : out std_logic;
    o_MemWrite    : out std_logic;
    o_WBSel       : out std_logic_vector(1 downto 0);
    o_LdByte      : out std_logic;
    o_LdHalf      : out std_logic;
    o_Halt        : out std_logic;
    o_LdUnsigned  : out std_logic
  );
end component;

--MEM/WB reg
component MEM_WB_reg is
  port(
    i_CLK       : in  std_logic;
    i_RST       : in  std_logic;
    i_EN        : in  std_logic;
    i_Halt      : in std_logic; 
    -- from MEM (M)
    PCPLUS4M    : in  std_logic_vector(31 downto 0);
    ALUResM     : in  std_logic_vector(31 downto 0);
    LoadDataM   : in  std_logic_vector(31 downto 0);
    RDM         : in  std_logic_vector(4 downto 0);
    RegWr_M     : in  std_logic;
    WBSel_M     : in  std_logic_vector(1 downto 0);
    -- to WB (W)
    PCPLUS4W    : out std_logic_vector(31 downto 0);
    ALUResW     : out std_logic_vector(31 downto 0);
    LoadDataW   : out std_logic_vector(31 downto 0);
    RDW         : out std_logic_vector(4 downto 0);
    RegWr_W     : out std_logic;
    o_Halt      : out std_logic;
    WBSel_W     : out std_logic_vector(1 downto 0)
  );
end component;


begin

  -- TODO: This is required to be your final input to your instruction memory. This provides a feasible method to externally load the memory module which means that the synthesis tool must assume it knows nothing about the values stored in the instruction memory. If this is not included, much, if not all of the design is optimized out because the synthesis tool will believe the memory to be all zeros.
  with iInstLd select
    s_IMemAddr <= s_NextInstAddr when '0',
      iInstAddr when others;


  IMem: mem
    generic map(ADDR_WIDTH => ADDR_WIDTH,
                DATA_WIDTH => N)
    port map(clk  => iCLK,
             addr => s_IMemAddr(11 downto 2),
             data => iInstExt,
             we   => iInstLd,
             q    => s_Inst);
  
  DMem: mem
    generic map(ADDR_WIDTH => ADDR_WIDTH,
                DATA_WIDTH => N)
    port map(clk  => iCLK,
             addr => s_DMemAddr(11 downto 2),
             data => s_DMemData,
             we   => s_DMemWr,
             q    => s_DMemOut);

  -- TODO: Ensure that s_Halt is connected to an output control signal produced from decoding the Halt instruction (Opcode: 01 0100)
  -- TODO: Ensure that s_Ovfl is connected to the overflow output of your ALU

  -- TODO: Implement the rest of your processor below this comment! 
s_InstrF <= s_Inst;
---------------------------
--IF/ID Decode stage
---------------------------
PCF_inst : PCFetch
  generic map(
    G_RESET_VECTOR => x"00000000"
  )
  port map(
    i_clk       => iCLK,
    i_rst       => iRST,
    i_halt      => s_Halt_W,
    i_pc_src    => PC_SEQ,
    i_br_taken  => '0',
    i_rs1_val   => (others => '0'),
    i_immI      => (others => '0'),
    i_immB      => (others => '0'),
    i_immJ      => (others => '0'),
    o_pc        => s_PCF,
    o_pc_plus4  => s_PCPlus4F,
    o_imem_addr => s_NextInstAddr
  );

-- Drive the required top-level s_Halt net:
s_Halt <= s_Halt_W;
-- IF/ID pipeline reg
IF_ID_pipe : IF_ID_Reg
  port map(
    i_CLK         => iCLK,
    i_RST         => iRST,
    i_WE          => '1',
    i_PC          => s_PCF,
    i_PC_Plus4    => s_PCPlus4F,
    i_Instruction => s_InstrF,
    o_PC          => s_PCD,
    o_PC_Plus4    => s_PCPlus4D,
    o_Instruction => s_InstrD
  );
 --Decode fields
  s_opcode <= s_InstrD(6  downto 0);
  s_rd     <= s_InstrD(11 downto 7);
  s_funct3 <= s_InstrD(14 downto 12);
  s_rs1    <= s_InstrD(19 downto 15);
  s_rs2    <= s_InstrD(24 downto 20);
  s_funct7 <= s_InstrD(31 downto 25);


------------------------
--ID/EX pipeline stage
------------------------

CU : ControlUnit
  port map(
    opcode      => s_opcode,
    funct3      => s_funct3,
    funct7      => s_funct7,
    imm12       => s_InstrD(31 downto 20),
    ALUSrc      => s_ALUSrcSel_D,
    ALUControl  => open,                  -- you keep this 2-bit if unused
    ImmType     => s_ImmKind_D,
    ResultSrc   => s_WBSel_D,
    MemWrite    => s_MemWrite_D,
    RegWrite    => s_RegWr_D,
    ALU_op      => s_AluCtrl_D,
    Halt        => s_Halt_D,
    MemRead     => s_MemRead_D,
    LdByte      => s_LdByte_D,
    LdHalf      => s_LdHalf_D,
    LdUnsigned  => s_LdUnsigned_D,
    ASel        => s_ASel,                -- you’ll pipe ASel later if needed
    Branch      => s_Branch_D,
    PCSrc       => PCSrc                  -- stays in IF for now
  );

RF : reg
  generic map(N => 32)
  port map(
    RS1      => s_rs1,
    RS2      => s_rs2,
    DATA_IN  => s_RegWrData,              -- existing WB path (will use E/M/W later)
    W_SEL    => s_RegWrAddr,
    WE       => s_RegWr_Final,
    RST      => iRST,
    CLK      => iCLK,
    RS1_OUT  => s_RS1_Val_D,
    RS2_OUT  => s_RS2_Val_D
  );

  IMMGEN : imm_generator
  port map(
    i_instr => s_InstrD,
    i_kind  => s_ImmKind_D,
    o_imm   => s_Imm_D
  );

  ID_EX : ID_EX_Reg
  generic map(
    DATA_WIDTH => N
  )
  port map(
    CLK           => iCLK,
    RST           => iRST,
    EN            => '1',
    PCD           => s_PCD,
    Funct3_D      => s_Funct3_D,
    ASel_D        => s_ASel_D,
    Funct3_E      => s_Funct3_E,
    ASel_E        => s_ASel_E,
    PCPLUS4D      => s_PCPlus4D,
    RD1D          => s_RS1_Val_D,
    RD2D          => s_RS2_Val_D,
    IMMD          => s_Imm_D,
    RS1D          => s_rs1,
    RS2D          => s_rs2,
    RDD           => s_rd,
    Halt_D        => s_Halt_D,
    LdUnsigned_D  => s_LdUnsigned_D,
    LdHalf_D      => s_LdHalf_D,
    LdByte_D      => s_LdByte_D,
    MemWrite_D    => s_MemWrite_D,
    MemRead_D     => s_MemRead_D,
    WBSel_D       => s_WBSel_D,
    Branch_D      => s_Branch_D,
    ImmKind_D     => s_ImmKind_D,
    RegWr_D       => s_RegWr_D,
    ALUSrcSel_D   => s_ALUSrcSel_D,
    AluCtrl_D     => s_AluCtrl_D,
    PCE           => s_PC_E,
    PCPLUS4E      => s_PCPlus4_E,
    RD1E          => s_RS1_Val_E,
    RD2E          => s_RS2_Val_E,
    IMME          => s_Imm_E,
    RS1E          => s_RS1_E,
    RS2E          => s_RS2_E,
    RDE           => s_RD_E,
    Halt_E        => s_Halt_E,
    LdUnsigned_E  => s_LdUnsigned_E,
    LdHalf_E      => s_LdHalf_E,
    LdByte_E      => s_LdByte_E,
    MemWrite_E    => s_MemWrite_E,
    MemRead_E     => s_MemRead_E,
    WBSel_E       => s_WBSel_E,
    Branch_E      => s_Branch_E,
    ImmKind_E     => s_ImmKind_E,
    RegWr_E       => s_RegWr_E,
    ALUSrcSel_E   => s_ALUSrcSel_E,
    AluCtrl_E     => s_AluCtrl_E
  );
oALUOut <= s_PC_E;  -- temporary; switch to ALU result later

---------------
--EX/MEM Stage!
---------------
s_ALUInA <= s_RS1_Val_E; --ALU input A is RS1 for now
-- ALU B input select: 0=RS2, 1=IMM
with s_ALUSrcSel_E select
  s_ALUInB <= s_RS2_Val_E when '0',
              s_Imm_E     when others;

ALU0 : ALUUnit
  port map(
    A         => s_ALUInA,
    B         => s_ALUInB,
    shift_amt => s_RS2_Val_E(4 downto 0),  -- if you use RS2[4:0] for shift count
    ALU_op    => s_AluCtrl_E,
    F         => s_ALURes,
    Zero      => s_ALUZero,
    Overflow  => s_ALUOvfl
  );
  EX_MEM : EX_MEM_reg
  port map(
    i_CLK        => iCLK,
    i_RST        => iRST,
    i_Halt       => s_Halt_E,
    i_WE         => '1',              -- no stalls yet
    i_PC_Plus4   => s_PCPlus4_E,
    i_ALU_Result => s_ALURes,
    i_WriteData  => s_RS2_Val_E,      -- store data comes from RS2 in EX
    i_RD         => s_RD_E,
    i_RegWrite   => s_RegWr_E,
    i_MemRead    => s_MemRead_E,
    i_MemWrite   => s_MemWrite_E,
    i_WBSel      => s_WBSel_E,
    i_LdByte     => s_LdByte_E,
    i_LdHalf     => s_LdHalf_E,
    i_LdUnsigned => s_LdUnsigned_E,
    o_PC_Plus4   => s_PCPlus4_M,
    o_ALU_Result => s_ALURes_M,
    o_WriteData  => s_WriteData_M,
    o_RD         => s_RD_M,
    o_RegWrite   => s_RegWr_M,
    o_MemRead    => s_MemRead_M,
    o_MemWrite   => s_MemWrite_M,
    o_WBSel      => s_WBSel_M,
    o_LdByte     => s_LdByte_M,
    o_LdHalf     => s_LdHalf_M,
    o_Halt       => s_Halt_M,
    o_LdUnsigned => s_LdUnsigned_M
  );
  --Memory portion
LSU : load_store_unit
  port map(
    i_addr        => s_ALURes_M,        -- byte address
    i_rs2_wdata   => s_WriteData_M,     -- store data
    i_mem_read    => s_MemRead_M,
    i_mem_write   => s_MemWrite_M,
    i_ld_byte     => s_LdByte_M,
    i_ld_half     => s_LdHalf_M,
    i_ld_unsigned => s_LdUnsigned_M,
    i_mem_rdata   => s_DMemOut,         -- from DMem
    o_mem_addr    => s_DMemAddr,
    o_mem_wdata   => s_DMemData,
    o_mem_we      => s_DMemWr,
    o_load_data   => s_LoadData_M
  );


-- TEMP WB path (until MEM/WB is added):
-- 00=ALU, 01=Load, 10=PC+4
with s_WBSel_W select
  s_WBData_W <= s_ALURes_M  when "00",
              s_LoadData_W when "01",
              s_PCPlus4_W  when "10",
              (others => '0') when others;

-- Final writeback controls come from WB stage now:
s_RegWr_Final <= s_RegWr_W;
s_RegWrAddr   <= s_RD_W;
s_RegWrData   <= s_WBData_W;


MEM_WB0 : MEM_WB_reg
  port map(
    i_CLK       => iCLK,
    i_RST       => iRST,
    i_EN        => '1',
    i_Halt      => s_Halt_M,               -- no stalling yet
    PCPLUS4M    => s_PCPlus4_M,
    ALUResM     => s_ALURes_M,
    LoadDataM   => s_LoadData_M,
    RDM         => s_RD_M,
    RegWr_M     => s_RegWr_M,
    WBSel_M     => s_WBSel_M,
    PCPLUS4W    => s_PCPlus4_W,
    ALUResW     => s_ALURes_W,
    LoadDataW   => s_LoadData_W,
    RDW         => s_RD_W,
    RegWr_W     => s_RegWr_W,
    o_Halt     => s_Halt_W,
    WBSel_W     => s_WBSel_W
  );

s_Halt <= s_Halt_W;  -- final halt signal to stop PC and stop testbench



end structure;