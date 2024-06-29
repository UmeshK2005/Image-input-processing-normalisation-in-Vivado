----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date: 05.11.2023 18:12:47
-- Design Name: 
-- Module Name: sim - Behavioral
-- Project Name: 
-- Target Devices: 
-- Tool Versions: 
-- Description: 
-- 
-- Dependencies: 
-- 
-- Revision:
-- Revision 0.01 - File Created
-- Additional Comments:
-- 
----------------------------------------------------------------------------------


library IEEE;
use IEEE.STD_LOGIC_1164.ALL;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx leaf cells in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity sim2 is
--  Port ( );
end sim2;

architecture Behavioral of sim2 is
signal reg00 : integer;
signal reg01 : integer;
signal reg02 : integer;
signal reg10 : integer;
signal reg11 : integer;
signal reg12 : integer;
signal reg20 : integer;
signal reg21 : integer;
signal reg22 : integer;
signal pix00 : integer;
signal pix01 : integer;
signal pix02 : integer;
signal pix10 : integer;
signal pix11 : integer;
signal pix12 : integer;
signal pix20 : integer;
signal pix21 : integer;
signal pix22 : integer;
signal data_in: std_logic_vector(15 downto 0);
signal nrm_data_in: std_logic_vector(15 downto 0);
signal rom_data: std_logic_vector(7 downto 0);
signal final_max_val: integer;
signal final_min_val: integer;
signal itr_nrml : integer;
signal jtr_nrml: integer;
signal i: integer;
signal j: integer;
signal nrm_write_enable: std_logic;
signal ram_address: std_logic_vector(11 downto 0);
signal nrm_ram_address: std_logic_vector(11 downto 0);
signal output_pixel: std_logic_vector(15 downto 0);
signal nrm_sum: integer;
signal clock : std_logic := '0';
constant clock_period : time := 10 ns;

component VGA_DRIVER1
port(    clk:in std_logic;
         hsync: out std_logic;
         vsync: out std_logic;   
         r: out std_logic_vector(3 downto 0);
         g: out std_logic_vector(3 downto 0);
         b: out std_logic_vector(3 downto 0);
         ram_data: out std_logic_vector(15 downto 0);
         rom_d: out std_logic_vector(7 downto 0);
         i1: out integer;
         j1: out integer;
         itr_nrml1: out integer;
         jtr_nrml1: out integer;
         nrm_data: out std_logic_vector(15 downto 0);
         nrm_wr_enable1: out std_logic;
         nrm_ram_add1: out std_logic_vector(11 downto 0);
         output_pixel1: out std_logic_vector(15 downto 0);
         nrm_sum1 : out integer;
         ram_add1: out std_logic_vector(11 downto 0);
         max_val: out integer;
         min_val:out integer;
         
        r00: out integer;
        r01: out integer;
        r02: out integer;
        r10: out integer;
        r11: out integer;
        r12: out integer;
        r20: out integer;
        r21: out integer;
        r22: out integer;
        i_00: out integer;
        i_01: out integer;
        i_02: out integer;
        i_10: out integer;
        i_11: out integer;
        i_12: out integer;
        i_20: out integer;
        i_21: out integer;
        i_22: out integer
         );
end component;

begin

u1: VGA_DRIVER1 port map(
        clk => clock,
        Hsync => open,
        Vsync => open,
        R => open,
        G => open,
        B => open,
        ram_data => data_in,
        rom_d => rom_data,

        i1 => i,
        j1=> j,
        itr_nrml1 => itr_nrml,
        jtr_nrml1 => jtr_nrml,
        nrm_data=>nrm_data_in,
        nrm_wr_enable1 => nrm_write_enable,
        nrm_ram_add1 => nrm_ram_address,
         output_pixel1 => output_pixel,
         nrm_sum1 => nrm_sum,
         ram_add1 => ram_address,
         max_val => final_max_val,
         min_val => final_min_val,
         r00=>reg00,
        r01=>reg01,
        r02=>reg02,
        r10=>reg10,
        r11=>reg11,
        r12=>reg12,
        r20=>reg20,
        r21=>reg21,
        r22=>reg22,
        i_00=>pix00,
        i_01=>pix01,
        i_02=>pix02,
        i_10=>pix10,
        i_11=>pix11,
        i_12=>pix12,
        i_20=>pix20,
        i_21=>pix21,
        i_22=>pix22
        );

clock_process :process
begin
    clock <= '0';
    wait for clock_period/2;
    clock <= '1';
    wait for clock_period/2;
end process;


end Behavioral;
