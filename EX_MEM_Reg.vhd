-- EX_MEM_reg (drop-in)
library ieee;
use ieee.std_logic_1164.all;

entity EX_MEM_reg is
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
end entity;

architecture rtl of EX_MEM_reg is
  -- Registered state with safe power-on defaults (prevents X at t=0)
  signal r_PC_Plus4   : std_logic_vector(31 downto 0) := (others => '0');
  signal r_ALU_Result : std_logic_vector(31 downto 0) := (others => '0');
  signal r_WriteData  : std_logic_vector(31 downto 0) := (others => '0');
  signal r_RD         : std_logic_vector(4 downto 0)  := (others => '0');
  signal r_RegWrite   : std_logic := '0';
  signal r_MemRead    : std_logic := '0';
  signal r_MemWrite   : std_logic := '0';
  signal r_WBSel      : std_logic_vector(1 downto 0) := "00";
  signal r_LdByte     : std_logic := '0';
  signal r_LdHalf     : std_logic := '0';
  signal r_LdUnsigned : std_logic := '0';
  signal r_Halt       : std_logic := '0';
begin
  -- Synchronous register. Treat ANY non-'0' reset ('1','U','X','Z',...) as asserted.
  process(i_CLK)
  begin
    if rising_edge(i_CLK) then
      if i_RST /= '0' then
        r_PC_Plus4   <= (others => '0');
        r_ALU_Result <= (others => '0');
        r_WriteData  <= (others => '0');
        r_RD         <= (others => '0');
        r_RegWrite   <= '0';
        r_MemRead    <= '0';
        r_MemWrite   <= '0';
        r_WBSel      <= "00";
        r_LdByte     <= '0';
        r_LdHalf     <= '0';
        r_LdUnsigned <= '0';
        r_Halt       <= '0';
      elsif i_WE = '1' then
        r_PC_Plus4   <= i_PC_Plus4;
        r_ALU_Result <= i_ALU_Result;
        r_WriteData  <= i_WriteData;
        r_RD         <= i_RD;
        r_RegWrite   <= i_RegWrite;
        r_MemRead    <= i_MemRead;
        r_MemWrite   <= i_MemWrite;
        r_WBSel      <= i_WBSel;
        r_LdByte     <= i_LdByte;
        r_LdHalf     <= i_LdHalf;
        r_LdUnsigned <= i_LdUnsigned;
        if i_Halt = '1' then
          r_Halt <= '1';
        else
          r_Halt <= '0';
        end if;

      end if;
    end if;
  end process;

  -- Outputs from the registered state (known from t=0)
  o_PC_Plus4   <= r_PC_Plus4;
  o_ALU_Result <= r_ALU_Result;
  o_WriteData  <= r_WriteData;
  o_RD         <= r_RD;
  o_RegWrite   <= r_RegWrite;
  o_MemRead    <= r_MemRead;
  o_MemWrite   <= r_MemWrite;
  o_WBSel      <= r_WBSel;
  o_LdByte     <= r_LdByte;
  o_LdHalf     <= r_LdHalf;
  o_LdUnsigned <= r_LdUnsigned;
  o_Halt       <= r_Halt;
end architecture;
