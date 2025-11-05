--Dylan Kramer and Michael Berg
--IF/ID Pipeline Register for Software-Scheduled RISC-V Processor
--Holds all values passing from Instruction Fetch (IF) to Instruction Decode (ID)
use work.RISCV_types.all;
library IEEE;
use IEEE.std_logic_1164.all;
use ieee.numeric_std.all;

entity IF_ID_reg is
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
end IF_ID_reg;

architecture structural of IF_ID_reg is

  -- Component declaration for N-bit register
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

  -- Internal signals for muxing between normal operation and NOP insertion
  signal s_PC_input         : std_logic_vector(31 downto 0);
  signal s_PC_Plus4_input   : std_logic_vector(31 downto 0);
  signal s_Instruction_input: std_logic_vector(31 downto 0);
  
  -- Current register outputs (for holding during stalls)
  signal s_PC_current         : std_logic_vector(31 downto 0);
  signal s_PC_Plus4_current   : std_logic_vector(31 downto 0);
  signal s_Instruction_current: std_logic_vector(31 downto 0);
  
  -- NOP instruction (addi x0, x0, 0) - used for hazard bubbles
  constant NOP : std_logic_vector(31 downto 0) := x"00000013";

begin

  -- Mux logic: select between stall (hold current) or normal operation
  -- When WE = '0', hold current values; when WE = '1', accept new values
  s_PC_input <= s_PC_current when (i_WE = '0') else i_PC;
  
  s_PC_Plus4_input <= s_PC_Plus4_current when (i_WE = '0') else i_PC_Plus4;
  
  -- For instruction: if not writing, keep current; if writing, pass through input
  -- Note: External hazard detection can also force NOP by setting i_Instruction to NOP
  s_Instruction_input <= s_Instruction_current when (i_WE = '0') else i_Instruction;

  -- PC Register
  PC_REG: N_reg
    generic map(N => 32)
    port map(
      Data_in  => s_PC_input,
      CLK      => i_CLK,
      WE       => '1',  -- Always enabled, mux handles stalling
      RST      => i_RST,
      Data_out => s_PC_current
    );
  o_PC <= s_PC_current;

  -- PC+4 Register
  PC_PLUS4_REG: N_reg
    generic map(N => 32)
    port map(
      Data_in  => s_PC_Plus4_input,
      CLK      => i_CLK,
      WE       => '1',  -- Always enabled, mux handles stalling
      RST      => i_RST,
      Data_out => s_PC_Plus4_current
    );
  o_PC_Plus4 <= s_PC_Plus4_current;

  -- Instruction Register
  INSTRUCTION_REG: N_reg
    generic map(N => 32)
    port map(
      Data_in  => s_Instruction_input,
      CLK      => i_CLK,
      WE       => '1',  -- Always enabled, mux handles stalling
      RST      => i_RST,
      Data_out => s_Instruction_current
    );
  o_Instruction <= s_Instruction_current;

end structural;