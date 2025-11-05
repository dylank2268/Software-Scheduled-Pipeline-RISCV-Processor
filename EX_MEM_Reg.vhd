-- Dylan Kramer and Michael Berg
-- EX/MEM Pipeline Register for Software-Scheduled RISC-V Processor
-- Holds values passing from Execution (EX) to Memory (MEM)

library IEEE;
use IEEE.std_logic_1164.all;
use work.RISCV_types.all;

entity EX_MEM_reg is
  port(
    i_CLK           : in  std_logic;
    i_RST           : in  std_logic;
    i_WE            : in  std_logic;  -- stall/advance

    -- Inputs from EX stage
    i_PC_Plus4      : in  std_logic_vector(31 downto 0); -- optional but helpful for JAL writeback
    i_ALU_Result    : in  std_logic_vector(31 downto 0);
    i_WriteData     : in  std_logic_vector(31 downto 0); -- RS2 data to store
    i_RD            : in  std_logic_vector(4 downto 0);

    -- Control inputs (must reach MEM/WB)
    i_RegWrite      : in  std_logic;
    i_MemRead       : in  std_logic;
    i_MemWrite      : in  std_logic;
    i_WBSel         : in  std_logic_vector(1 downto 0);
    i_LdByte        : in  std_logic;
    i_LdHalf        : in  std_logic;
    i_LdUnsigned    : in  std_logic;
    i_Halt          : in std_logic;

    -- Outputs to MEM stage
    o_PC_Plus4      : out std_logic_vector(31 downto 0);
    o_ALU_Result    : out std_logic_vector(31 downto 0);
    o_WriteData     : out std_logic_vector(31 downto 0);
    o_RD            : out std_logic_vector(4 downto 0);

    o_RegWrite      : out std_logic;
    o_MemRead       : out std_logic;
    o_MemWrite      : out std_logic;
    o_WBSel         : out std_logic_vector(1 downto 0);
    o_LdByte        : out std_logic;
    o_LdHalf        : out std_logic;
    o_Halt          : out std_logic;
    
    o_LdUnsigned    : out std_logic
  );
end EX_MEM_reg;

architecture structural of EX_MEM_reg is

  -- Use the real N_reg entity (Data_in/WE/Data_out)
  component N_reg is
    generic(N : integer := 32);
    port(
      Data_in  : in  std_logic_vector(N-1 downto 0);
      CLK      : in  std_logic;
      WE       : in  std_logic;
      RST      : in  std_logic;
      Data_out : out std_logic_vector(N-1 downto 0)
    );
  end component;

  -- 1-bit vectors for clean mapping to N_reg
  signal RegWrite_v   : std_logic_vector(0 downto 0);
  signal MemRead_v    : std_logic_vector(0 downto 0);
  signal MemWrite_v   : std_logic_vector(0 downto 0);
  signal LdByte_v     : std_logic_vector(0 downto 0);
  signal LdHalf_v     : std_logic_vector(0 downto 0);
  signal LdUnsigned_v : std_logic_vector(0 downto 0);

begin
  -- ===== Data =====

  PC4_REG : entity work.N_reg
    generic map(N => 32)
    port map(
      Data_in  => i_PC_Plus4,
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => o_PC_Plus4
    );

  ALU_REG : entity work.N_reg
    generic map(N => 32)
    port map(
      Data_in  => i_ALU_Result,
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => o_ALU_Result
    );

  WD_REG : entity work.N_reg
    generic map(N => 32)
    port map(
      Data_in  => i_WriteData,
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => o_WriteData
    );

  RD_REG : entity work.N_reg
    generic map(N => 5)
    port map(
      Data_in  => i_RD,
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => o_RD
    );

  -- ===== Control =====

  REGWRITE_REG : entity work.N_reg
    generic map(N => 1)
    port map(
      Data_in  => (0 => i_RegWrite),
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => RegWrite_v
    );
  o_RegWrite <= RegWrite_v(0);

  MEMREAD_REG : entity work.N_reg
    generic map(N => 1)
    port map(
      Data_in  => (0 => i_MemRead),
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => MemRead_v
    );
  o_MemRead <= MemRead_v(0);

  MEMWRITE_REG : entity work.N_reg
    generic map(N => 1)
    port map(
      Data_in  => (0 => i_MemWrite),
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => MemWrite_v
    );
  o_MemWrite <= MemWrite_v(0);

  WBSEL_REG : entity work.N_reg
    generic map(N => 2)
    port map(
      Data_in  => i_WBSel,
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => o_WBSel
    );

  LDB_REG : entity work.N_reg
    generic map(N => 1)
    port map(
      Data_in  => (0 => i_LdByte),
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => LdByte_v
    );
  o_LdByte <= LdByte_v(0);

  LDH_REG : entity work.N_reg
    generic map(N => 1)
    port map(
      Data_in  => (0 => i_LdHalf),
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => LdHalf_v
    );
  o_LdHalf <= LdHalf_v(0);

  LDU_REG : entity work.N_reg
    generic map(N => 1)
    port map(
      Data_in  => (0 => i_LdUnsigned),
      CLK      => i_CLK,
      WE       => i_WE,
      RST      => i_RST,
      Data_out => LdUnsigned_v
    );
  o_LdUnsigned <= LdUnsigned_v(0);

end structural;
