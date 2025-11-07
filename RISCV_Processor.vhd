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
signal s_LoadedData : std_logic_vector(31 downto 0);
signal s_RegWrLoad : std_logic;
signal s_RegWr_Final : std_logic;

--Signals for AUIPC logic
signal s_ALUInA : std_logic_vector(31 downto 0);
signal s_ASel   : std_logic_vector(1 downto 0); -- 00=RS1, 01=PC, 10=ZERO

--Signals for branch logic
signal s_Branch : std_logic; --From control unit
signal s_BranchTaken : std_logic; --From branch logic 

--Signals for Decode propagation
signal s_IF_ID_PC : std_logic_vector(31 downto 0);
signal s_IF_ID_PCPlus4 : std_logic_vector(31 downto 0);
signal s_IF_ID_Inst : std_logic_vector(31 downto 0);


--Signals for ID/EX Pipeline propagation
-- ID/EX Pipeline Register Signals (Data Path)
signal s_ID_EX_PC       : std_logic_vector(31 downto 0);
signal s_ID_EX_PCPlus4  : std_logic_vector(31 downto 0);
signal s_ID_EX_RD1      : std_logic_vector(31 downto 0);  -- RS1 value
signal s_ID_EX_RD2      : std_logic_vector(31 downto 0);  -- RS2 value
signal s_ID_EX_IMM      : std_logic_vector(31 downto 0);  -- Immediate
signal s_ID_EX_RS1      : std_logic_vector(4 downto 0);   -- RS1 address
signal s_ID_EX_RS2      : std_logic_vector(4 downto 0);   -- RS2 address
signal s_ID_EX_RD       : std_logic_vector(4 downto 0);   -- RD address
-- ID/EX Pipeline Register Signals (Control Path)
signal s_ID_EX_Funct3      : std_logic_vector(2 downto 0);
signal s_ID_EX_ASel        : std_logic_vector(1 downto 0);
signal s_ID_EX_Halt        : std_logic;
signal s_ID_EX_LdUnsigned  : std_logic;
signal s_ID_EX_LdHalf      : std_logic;
signal s_ID_EX_LdByte      : std_logic;
signal s_ID_EX_MemWrite    : std_logic;
signal s_ID_EX_MemRead     : std_logic;
signal s_ID_EX_WBSel       : std_logic_vector(1 downto 0);
signal s_ID_EX_Branch      : std_logic;
signal s_ID_EX_ImmKind     : std_logic_vector(2 downto 0);
signal s_ID_EX_RegWr       : std_logic;
signal s_ID_EX_ALUSrcSel   : std_logic;
signal s_ID_EX_ALUCtrl     : std_logic_vector(3 downto 0);

--EX/MEM pipeline register signals 
signal s_EX_MEM_Halt        : std_logic;
signal s_EX_MEM_PCPlus4     : std_logic_vector(31 downto 0);
signal s_EX_MEM_ALURes      : std_logic_vector(31 downto 0);
signal s_EX_MEM_WriteData   : std_logic_vector(31 downto 0);
signal s_EX_MEM_RD          : std_logic_vector(4 downto 0);
signal s_EX_MEM_RegWr       : std_logic;
signal s_EX_MEM_MemRead     : std_logic;
signal s_EX_MEM_MemWrite    : std_logic;
signal s_EX_MEM_WBSel       : std_logic_vector(1 downto 0);
signal s_EX_MEM_LdByte      : std_logic;
signal s_EX_MEM_LdHalf      : std_logic;
signal s_EX_MEM_LdUnsigned  : std_logic;


--MEM/WB pipeline reg signals
  signal s_PCPLUS4_W      :std_logic_vector(31 downto 0);
  signal s_ALURes_W       :std_logic_vector(31 downto 0);
  signal s_LoadData_W     :std_logic_vector(31 downto 0);
  signal s_RD_W           :std_logic_vector(4 downto 0);
  signal s_RegWr_W        :std_logic;
  signal s_Halt_W         :std_logic;
  signal s_WBSel_W        :std_logic_vector(1 downto 0);

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
--IF/ID pipeline reg instantiation
component IF_ID_Reg is
    port(
    i_CLK        : in  std_logic;
    i_RST        : in  std_logic;
    i_WE         : in  std_logic;  -- Write enable (for stalling the pipeline)
    -- Inputs from IF stage
    i_PC         : in  std_logic_vector(31 downto 0);
    i_PC_Plus4   : in  std_logic_vector(31 downto 0);
    i_Instruction: in  std_logic_vector(31 downto 0);
    -- Outputs to ID stage
    o_PC         : out std_logic_vector(31 downto 0);
    o_PC_Plus4   : out std_logic_vector(31 downto 0);
    o_Instruction: out std_logic_vector(31 downto 0)
  );
  end component;
--ID_EX pipeline reg instantiation 
component ID_EX_reg is
  generic(
    DATA_WIDTH : integer := 32
  );
  port(
    CLK           : in  std_logic;
    RST           : in  std_logic;
    EN            : in  std_logic;
    -- Data from D
    PCD           : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    PCPLUS4D      : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    RD1D          : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    RD2D          : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    IMMD          : in  std_logic_vector(DATA_WIDTH-1 downto 0);
    RS1D          : in  std_logic_vector(4 downto 0);
    RS2D          : in  std_logic_vector(4 downto 0);
    RDD           : in  std_logic_vector(4 downto 0);
    Funct3_D      : in  std_logic_vector(2 downto 0);
    ASel_D        : in  std_logic_vector(1 downto 0);
    -- Control (D)
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
    -- Data to E
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
    -- Control to E
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
--EX/MEM pipeline reg instantiation
component EX_MEM_Reg is
    port(
    i_CLK         : in  std_logic;
    i_RST         : in  std_logic;
    i_WE          : in  std_logic;
    i_Halt        : in  std_logic;
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
  --MEM/WB Pipeline reg instantiation
component MEM_WB_Reg is
  port(
    i_CLK          : in  std_logic;
    i_RST          : in  std_logic;
    i_EN           : in  std_logic;  -- stall: '0' holds current W values
    i_Halt         : in std_logic; 
    -- Inputs from MEM stage (M)
    PCPLUS4M       : in  std_logic_vector(31 downto 0);
    ALUResM        : in  std_logic_vector(31 downto 0);
    LoadDataM      : in  std_logic_vector(31 downto 0);
    RDM            : in  std_logic_vector(4 downto 0);
    RegWr_M        : in  std_logic;
    WBSel_M        : in  std_logic_vector(1 downto 0);
    -- Outputs to WB stage (W)
    PCPLUS4W       : out std_logic_vector(31 downto 0);
    ALUResW        : out std_logic_vector(31 downto 0);
    LoadDataW      : out std_logic_vector(31 downto 0);
    RDW            : out std_logic_vector(4 downto 0);
    RegWr_W        : out std_logic;
    o_Halt         : out std_logic;
    WBSel_W        : out std_logic_vector(1 downto 0)
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
  -- ============================================================
  -- IF STAGE: PC FETCH UNIT
  -- ============================================================
  PCU: PCFetch
    generic map(G_RESET_VECTOR => x"00400000")
    port map(
      i_clk       => iCLK,
      i_rst       => iRST,
      i_halt      => s_Halt,
      i_pc_src    => PCSrc,
      i_br_taken  => s_BranchTaken,
      i_rs1_val   => s_rs1_val,
      i_immI      => s_immI,
      i_immB      => s_immB,
      i_immJ      => s_immJ,
      o_pc        => s_PC,
      o_pc_plus4  => s_PCPlus4,
      o_imem_addr => s_NextInstAddr
    );

  -- IF/ID PIPELINE REGISTER
  IF_ID: IF_ID_Reg
    port map(
      i_CLK         => iCLK,
      i_RST         => iRST,
      i_WE          => '1',  -- Always enabled (no stalls in basic design)
      -- Inputs from IF stage
      i_PC          => s_PC,
      i_PC_Plus4    => s_PCPlus4,
      i_Instruction => s_Inst,
      -- Outputs to ID stage
      o_PC          => s_IF_ID_PC,
      o_PC_Plus4    => s_IF_ID_PCPlus4,
      o_Instruction => s_IF_ID_Inst
    );

  -- ID STAGE: INSTRUCTION DECODE
  -- Extract instruction fields from pipelined instruction
  s_opcode <= s_IF_ID_Inst(6  downto 0);
  s_rd     <= s_IF_ID_Inst(11 downto 7);
  s_funct3 <= s_IF_ID_Inst(14 downto 12);
  s_rs1    <= s_IF_ID_Inst(19 downto 15);
  s_rs2    <= s_IF_ID_Inst(24 downto 20);
  s_funct7 <= s_IF_ID_Inst(31 downto 25);

  -- Destination register (rd) for regfile writeback
  s_RegWrAddr <= s_rd;

  -- Shift amount calculation
  s_ALUShiftAmt <= s_rs2_val(4 downto 0) when (s_opcode = "0110011" and (s_funct3 = "001" or s_funct3 = "101")) else
                   s_IF_ID_Inst(24 downto 20) when (s_opcode = "0010011" and (s_funct3 = "001" or s_funct3 = "101")) else
                   (others => '0');

  -- ID STAGE: CONTROL UNIT
  U_CTRL: ControlUnit
    port map(
      opcode     => s_opcode,
      funct3     => s_funct3,
      funct7     => s_funct7,
      imm12      => s_IF_ID_Inst(31 downto 20),
      ALUSrc     => s_ALUSrcSel,
      ALUControl => open,
      ImmType    => s_ImmKind,
      ResultSrc  => s_WBSel,
      MemWrite   => s_MemWrite,
      RegWrite   => s_RegWr,
      ALU_op     => s_ALUCtrl,
      Halt       => s_Halt,
      MemRead    => s_MemRead,
      LdByte     => s_LdByte,
      LdHalf     => s_LdHalf,
      LdUnsigned => s_LdUnsigned,
      ASel       => s_ASel,
      Branch     => s_Branch,
      PCSrc      => PCSrc
    );

  -- ID STAGE: IMMEDIATE GENERATORS
  IMM_I: imm_generator
    port map(
      i_instr => s_IF_ID_Inst,
      i_kind  => s_ImmKind,
      o_imm   => s_immI
    );

  IMM_B: imm_generator
    port map(
      i_instr => s_IF_ID_Inst,
      i_kind  => s_ImmKind,
      o_imm   => s_immB
    );

  IMM_J: imm_generator
    port map(
      i_instr => s_IF_ID_Inst,
      i_kind  => s_ImmKind,
      o_imm   => s_immJ
    );

  -- ID STAGE: REGISTER FILE
  REG_file: reg
    port map(
      RS1     => s_rs1,
      RS2     => s_rs2,
      DATA_IN => s_RegWrData,
      W_SEL   => s_RegWrAddr,
      WE      => s_RegWr,
      RST     => iRST,
      CLK     => iCLK,
      RS1_OUT => s_rs1_val,
      RS2_OUT => s_rs2_val
    );

 ID_EX_Pipe: ID_EX_Reg
  port map( 
    -- Clock and Control
    CLK           => iCLK,
    RST           => iRST,
    EN            => '1',  -- Always enabled (no stalls in basic design)
    
    -- Data Path Inputs from ID Stage (D)
    PCD           => s_IF_ID_PC,
    PCPLUS4D      => s_IF_ID_PCPlus4,
    RD1D          => s_rs1_val,      -- RS1 register value
    RD2D          => s_rs2_val,      -- RS2 register value
    IMMD          => s_immI,         -- Immediate value (use one immediate generator output)
    RS1D          => s_rs1,          -- RS1 address
    RS2D          => s_rs2,          -- RS2 address
    RDD           => s_rd,           -- RD address
    Funct3_D      => s_funct3,
    ASel_D        => s_ASel,
    
    -- Control Signals from ID Stage (D)
    Halt_D        => s_Halt,
    LdUnsigned_D  => s_LdUnsigned,
    LdHalf_D      => s_LdHalf,
    LdByte_D      => s_LdByte,
    MemWrite_D    => s_MemWrite,
    MemRead_D     => s_MemRead,
    WBSel_D       => s_WBSel,
    Branch_D      => s_Branch,
    ImmKind_D     => s_ImmKind,
    RegWr_D       => s_RegWr,
    ALUSrcSel_D   => s_ALUSrcSel,
    AluCtrl_D     => s_ALUCtrl,
    
    -- Data Path Outputs to EX Stage (E)
    PCE           => s_ID_EX_PC,
    PCPLUS4E      => s_ID_EX_PCPlus4,
    RD1E          => s_ID_EX_RD1,
    RD2E          => s_ID_EX_RD2,
    IMME          => s_ID_EX_IMM,
    RS1E          => s_ID_EX_RS1,
    RS2E          => s_ID_EX_RS2,
    RDE           => s_ID_EX_RD,
    Funct3_E      => s_ID_EX_Funct3,
    ASel_E        => s_ID_EX_ASel,
    
    -- Control Outputs to EX Stage (E)
    Halt_E        => s_ID_EX_Halt,
    LdUnsigned_E  => s_ID_EX_LdUnsigned,
    LdHalf_E      => s_ID_EX_LdHalf,
    LdByte_E      => s_ID_EX_LdByte,
    MemWrite_E    => s_ID_EX_MemWrite,
    MemRead_E     => s_ID_EX_MemRead,
    WBSel_E       => s_ID_EX_WBSel,
    Branch_E      => s_ID_EX_Branch,
    ImmKind_E     => s_ID_EX_ImmKind,
    RegWr_E       => s_ID_EX_RegWr,
    ALUSrcSel_E   => s_ID_EX_ALUSrcSel,
    AluCtrl_E     => s_ID_EX_ALUCtrl
  );



  -- EX STAGE: ALU INPUT SELECTION
  -- ALU Input A Mux (for AUIPC, LUI, normal ops)
  with s_ASel select
    s_ALUInA <= s_rs1_val       when "00",  -- Normal: RS1
                s_IF_ID_PC      when "01",  -- AUIPC: use pipelined PC
                (others => '0') when "10",  -- LUI: zero
                s_rs1_val       when others;

  -- ALU Input B Mux (RS2 or Immediate)
  ALU_B_MUX: mux2t1_N
    generic map(N => 32)
    port map(
      i_S  => s_ALUSrcSel,
      i_D0 => s_rs2_val,
      i_D1 => s_immI,
      o_O  => s_ALUInB
    );

  -- EX STAGE: ALU
  ALU: ALUUnit
    port map(
      A         => s_ALUInA,
      B         => s_ALUInB,
      shift_amt => s_ALUShiftAmt,
      ALU_op    => s_ALUCtrl,
      F         => s_ALURes,
      Zero      => s_ALUZero,
      Overflow  => s_Ovfl
    );

  -- Connect ALU output to required output port
  oALUOut <= s_ALURes;
  s_Ovfl <= s_ALUOvfl;
 

  -- EX STAGE: BRANCH LOGIC
  BRANCH_UNIT: branch_logic
    port map(
      i_rs1      => s_rs1_val,
      i_rs2      => s_rs2_val,
      i_funct3   => s_funct3,
      i_branch   => s_Branch,
      o_br_taken => s_BranchTaken
    );

EX_MEM_Pipe: EX_MEM_reg
  port map(
    -- Clock and Control
    i_CLK         => iCLK,
    i_RST         => iRST,
    i_WE          => '1',  -- Always enabled (no stalls in basic design)

    -- Data Path Inputs from EX Stage
    i_Halt        => s_ID_EX_Halt,
    i_PC_Plus4    => s_ID_EX_PCPlus4,
    i_ALU_Result  => s_ALURes,
    i_WriteData   => s_ID_EX_RD2,      -- RS2 value for store instructions
    i_RD          => s_ID_EX_RD,

    -- Control Signals from EX Stage
    i_RegWrite    => s_ID_EX_RegWr,
    i_MemRead     => s_ID_EX_MemRead,
    i_MemWrite    => s_ID_EX_MemWrite,
    i_WBSel       => s_ID_EX_WBSel,
    i_LdByte      => s_ID_EX_LdByte,
    i_LdHalf      => s_ID_EX_LdHalf,
    i_LdUnsigned  => s_ID_EX_LdUnsigned,

    -- Data Path Outputs to MEM Stage
    o_Halt        => s_EX_MEM_Halt,
    o_PC_Plus4    => s_EX_MEM_PCPlus4,
    o_ALU_Result  => s_EX_MEM_ALURes,
    o_WriteData   => s_EX_MEM_WriteData,
    o_RD          => s_EX_MEM_RD,

    -- Control Outputs to MEM Stage
    o_RegWrite    => s_EX_MEM_RegWr,
    o_MemRead     => s_EX_MEM_MemRead,
    o_MemWrite    => s_EX_MEM_MemWrite,
    o_WBSel       => s_EX_MEM_WBSel,
    o_LdByte      => s_EX_MEM_LdByte,
    o_LdHalf      => s_EX_MEM_LdHalf,
    o_LdUnsigned  => s_EX_MEM_LdUnsigned
  );


  -- MEM STAGE: LOAD/STORE UNIT
  LSU: load_store_unit
    port map(
      i_addr        => s_ALURes,
      i_rs2_wdata   => s_rs2_val,
      i_mem_read    => s_MemRead,
      i_mem_write   => s_MemWrite,
      i_ld_byte     => s_LdByte,
      i_ld_half     => s_LdHalf,
      i_ld_unsigned => s_LdUnsigned,
      i_mem_rdata   => s_DMemOut,
      o_mem_addr    => s_DMemAddr,
      o_mem_wdata   => s_DMemData,
      o_mem_we      => s_DMemWr,
      o_load_data   => s_LoadedData
    );


  MEM_WB_Pipe: MEM_WB_Reg
    port map(
      i_CLK => iCLK,
      i_RST => iRST,
      i_EN => '1',
      i_Halt => s_EX_MEM_Halt,
      PCPLUS4M => s_EX_MEM_PCPlus4,
      ALUResM => s_EX_MEM_ALURes,
      LoadDataM => s_EX_MEM_WriteData,
      RDM => s_EX_MEM_RD,
      RegWr_M => s_EX_MEM_RegWr,
      WBSel_M => s_EX_MEM_WBSel,
      PCPlus4W => s_PCPLUS4_W,
      ALUResW => s_ALURes_W,
      LoadDataW => s_LoadData_W,
      RDW => s_RD_W,
      RegWr_W => s_RegWr_W,
      o_Halt => s_Halt_W,
      WBSel_W => s_WBSel_W
    );
  
  -- ============================================================
  -- WB STAGE: WRITEBACK MUX
  -- ============================================================
  WB_MUX: mux4t1_N
    generic map(N => 32)
    port map(
      i_S  => s_WBSel,
      i_D0 => s_ALURes,      -- 00: ALU result
      i_D1 => s_LoadedData,  -- 01: Load data from memory
      i_D2 => s_IF_ID_PCPlus4, -- 10: PC+4 (for JAL/JALR)
      i_D3 => (others => '0'), -- 11: unused
      o_O  => s_RegWrData
    );




end structure;