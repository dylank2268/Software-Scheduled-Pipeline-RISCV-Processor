-- Dylan Kramer and Michael Berg
-- MEM/WB Pipeline Register (M → W) with D/E/M/W naming convention

library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;
use work.RISCV_types.all;

entity MEM_WB_reg is
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
end MEM_WB_reg;

architecture structural of MEM_WB_reg is
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

  -- Hold (stall) muxes
  signal PCPLUS4M_in, PCPLUS4W_q  : std_logic_vector(31 downto 0);
  signal ALUResM_in,  ALUResW_q   : std_logic_vector(31 downto 0);
  signal LoadDataM_in, LoadDataW_q: std_logic_vector(31 downto 0);
  signal RDM_in, RDW_q            : std_logic_vector(4 downto 0);
  signal RegWr_M_in               : std_logic;
  signal WBSel_M_in, WBSel_W_q    : std_logic_vector(1 downto 0);

  signal RegWr_W_q_v : std_logic_vector(0 downto 0);

begin
  -- Stall selection
  PCPLUS4M_in  <= PCPLUS4W_q   when (i_EN = '0') else PCPLUS4M;
  ALUResM_in   <= ALUResW_q    when (i_EN = '0') else ALUResM;
  LoadDataM_in <= LoadDataW_q  when (i_EN = '0') else LoadDataM;
  RDM_in       <= RDW_q        when (i_EN = '0') else RDM;
  RegWr_M_in   <= RegWr_W_q_v(0) when (i_EN = '0') else RegWr_M;
  WBSel_M_in   <= WBSel_W_q    when (i_EN = '0') else WBSel_M;

  PC4_REG : N_reg
    generic map(N => 32)
    port map(Data_in => PCPLUS4M_in, CLK => i_CLK, WE => '1', RST => i_RST, Data_out => PCPLUS4W_q);
  PCPLUS4W <= PCPLUS4W_q;

  ALU_REG : N_reg
    generic map(N => 32)
    port map(Data_in => ALUResM_in, CLK => i_CLK, WE => '1', RST => i_RST, Data_out => ALUResW_q);
  ALUResW <= ALUResW_q;

  LD_REG : N_reg
    generic map(N => 32)
    port map(Data_in => LoadDataM_in, CLK => i_CLK, WE => '1', RST => i_RST, Data_out => LoadDataW_q);
  LoadDataW <= LoadDataW_q;

  RD_REG : N_reg
    generic map(N => 5)
    port map(Data_in => RDM_in, CLK => i_CLK, WE => '1', RST => i_RST, Data_out => RDW_q);
  RDW <= RDW_q;

  REGWR_REG : N_reg
    generic map(N => 1)
    port map(Data_in(0) => RegWr_M_in, CLK => i_CLK, WE => '1', RST => i_RST, Data_out => RegWr_W_q_v);
  RegWr_W <= RegWr_W_q_v(0);

  WBSEL_REG : N_reg
    generic map(N => 2)
    port map(Data_in => WBSel_M_in, CLK => i_CLK, WE => '1', RST => i_RST, Data_out => WBSel_W_q);
  WBSel_W <= WBSel_W_q;

end structural;
