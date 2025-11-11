-- Dylan Kramer
-- ID/EX stage pipeline Reg (+Funct3, +ASel)
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use work.RISCV_types.all;

entity ID_EX_reg is
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
    INST_D      : in std_logic_vector(31 downto 0);

    -- NEW control (D)
    Funct3_D      : in  std_logic_vector(2 downto 0);  -- for branch compares
    ASel_D        : in  std_logic_vector(1 downto 0);  -- 00=RS1,01=PC,10=ZERO

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
    PCSrc_D       : in pc_src_t;

    -- Data to E
    PCE           : out std_logic_vector(DATA_WIDTH-1 downto 0);
    PCPLUS4E      : out std_logic_vector(DATA_WIDTH-1 downto 0);
    RD1E          : out std_logic_vector(DATA_WIDTH-1 downto 0);
    RD2E          : out std_logic_vector(DATA_WIDTH-1 downto 0);
    IMME          : out std_logic_vector(DATA_WIDTH-1 downto 0);
    RS1E          : out std_logic_vector(4 downto 0);
    RS2E          : out std_logic_vector(4 downto 0);
    RDE           : out std_logic_vector(4 downto 0);
    PCSrc_E       : out pc_src_t;

    -- NEW control (E)
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
    EX_INST       : out std_logic_vector(31 downto 0);
    INST_E        : out std_logic_vector(31 downto 0);
    AluCtrl_E     : out std_logic_vector(3 downto 0)
  );
end entity;

architecture simple of ID_EX_reg is
  -- 1-bit vectors for casting
  signal HaltE_v        : std_logic_vector(0 downto 0);
  signal LdUnsigned_E_v : std_logic_vector(0 downto 0);
  signal LdHalf_E_v     : std_logic_vector(0 downto 0);
  signal LdByte_E_v     : std_logic_vector(0 downto 0);
  signal MemWrite_E_v   : std_logic_vector(0 downto 0);
  signal MemRead_E_v    : std_logic_vector(0 downto 0);
  signal Branch_E_v     : std_logic_vector(0 downto 0);
  signal RegWr_E_v      : std_logic_vector(0 downto 0);
  signal ALUSrcSel_E_v  : std_logic_vector(0 downto 0);
begin
  -- ============ data ============
  PCE_reg : entity work.N_reg
    generic map(N => DATA_WIDTH)
    port map(Data_in => PCD, CLK => CLK, WE => EN, RST => RST, Data_out => PCE);

  PC4_reg : entity work.N_reg
    generic map(N => DATA_WIDTH)
    port map(Data_in => PCPLUS4D, CLK => CLK, WE => EN, RST => RST, Data_out => PCPLUS4E);

  RD1_reg : entity work.N_reg
    generic map(N => DATA_WIDTH)
    port map(Data_in => RD1D, CLK => CLK, WE => EN, RST => RST, Data_out => RD1E);

  RD2_reg : entity work.N_reg
    generic map(N => DATA_WIDTH)
    port map(Data_in => RD2D, CLK => CLK, WE => EN, RST => RST, Data_out => RD2E);

  IMM_reg : entity work.N_reg
    generic map(N => DATA_WIDTH)
    port map(Data_in => IMMD, CLK => CLK, WE => EN, RST => RST, Data_out => IMME);

  RS1_reg : entity work.N_reg
    generic map(N => 5)
    port map(Data_in => RS1D, CLK => CLK, WE => EN, RST => RST, Data_out => RS1E);

  RS2_reg : entity work.N_reg
    generic map(N => 5)
    port map(Data_in => RS2D, CLK => CLK, WE => EN, RST => RST, Data_out => RS2E);

  RD_reg : entity work.N_reg
    generic map(N => 5)
    port map(Data_in => RDD, CLK => CLK, WE => EN, RST => RST, Data_out => RDE);

  INST_reg : entity work.N_reg
    generic map(N => 32)
    port map(Data_in => INST_D, CLK => CLK, WE => EN, RST => RST, Data_out => INST_E);



  -- ============ NEW control pipes ============
  FUNCT3_reg : entity work.N_reg
    generic map(N => 3)
    port map(
      Data_in  => Funct3_D,
      CLK      => CLK,
      WE       => EN,
      RST      => RST,
      Data_out => Funct3_E
    );

  ASEL_reg : entity work.N_reg
    generic map(N => 2)
    port map(
      Data_in  => ASel_D,
      CLK      => CLK,
      WE       => EN,
      RST      => RST,
      Data_out => ASel_E
    );

  Halt_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => Halt_D), CLK => CLK, WE => EN, RST => RST, Data_out => HaltE_v);

  LDU_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => LdUnsigned_D), CLK => CLK, WE => EN, RST => RST, Data_out => LdUnsigned_E_v);

  LDH_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => LdHalf_D), CLK => CLK, WE => EN, RST => RST, Data_out => LdHalf_E_v);

  LDB_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => LdByte_D), CLK => CLK, WE => EN, RST => RST, Data_out => LdByte_E_v);

  MW_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => MemWrite_D), CLK => CLK, WE => EN, RST => RST, Data_out => MemWrite_E_v);

  MR_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => MemRead_D), CLK => CLK, WE => EN, RST => RST, Data_out => MemRead_E_v);

  WBS_reg : entity work.N_reg
    generic map(N => 2)
    port map(Data_in => WBSel_D, CLK => CLK, WE => EN, RST => RST, Data_out => WBSel_E);

  BR_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => Branch_D), CLK => CLK, WE => EN, RST => RST, Data_out => Branch_E_v);

  IMMK_reg : entity work.N_reg
    generic map(N => 3)
    port map(Data_in => ImmKind_D, CLK => CLK, WE => EN, RST => RST, Data_out => ImmKind_E);

  REGWR_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => RegWr_D), CLK => CLK, WE => EN, RST => RST, Data_out => RegWr_E_v);

  ALUSRCSEL_reg : entity work.N_reg
    generic map(N => 1)
    port map(Data_in => (0 => ALUSrcSel_D), CLK => CLK, WE => EN, RST => RST, Data_out => ALUSrcSel_E_v);

  ALUCTRL_reg : entity work.N_reg
    generic map(N => 4)
    port map(Data_in => AluCtrl_D, CLK => CLK, WE => EN, RST => RST, Data_out => AluCtrl_E);

  -- ============ cast 1-bit vectors ============
  Halt_E       <= HaltE_v(0);
  LdUnsigned_E <= LdUnsigned_E_v(0);
  LdHalf_E     <= LdHalf_E_v(0);
  LdByte_E     <= LdByte_E_v(0);
  MemWrite_E   <= MemWrite_E_v(0);
  MemRead_E    <= MemRead_E_v(0);
  Branch_E     <= Branch_E_v(0);
  RegWr_E      <= RegWr_E_v(0);
  ALUSrcSel_E  <= ALUSrcSel_E_v(0);
end architecture;