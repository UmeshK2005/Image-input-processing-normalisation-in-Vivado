library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity VGA_DRIVER1 is
Port(
    clk:in std_logic;
    Hsync: out std_logic;
    Vsync: out std_logic;
    R: out std_logic_vector(3 downto 0);
    G: out std_logic_vector(3 downto 0);
    B: out std_logic_vector(3 downto 0)
);
end VGA_DRIVER1;


architecture Behavioral of VGA_DRIVER1 is

signal clk_slow: std_logic:='0';
signal FSM_state: integer := 0;
signal kernel_itr:integer :=0;
signal kernel_comp: integer:=0;
signal ker_rom_add: std_logic_vector(3 downto 0) := (others => '0');
signal data_rom_ker: std_logic_vector(7 downto 0) := (others => '0');
signal final_min_val: integer :=65536;
signal output_pixel : std_logic_vector(15 downto 0):= (others =>'0');
signal data_in: std_logic_vector(15 downto 0):= (others => '0');
signal final_max_val: integer := -65536;
signal img_counter: integer := 0;
signal itr_nrml,jtr_nrml:integer :=0;
signal nrm_comp: integer :=0;
signal nrm_ram_add: std_logic_vector(11 downto 0):=(others =>'0');
signal nrm_data_rom: std_logic_vector(7 downto 0) := (others => '0');
signal nrm_output_pixel : std_logic_vector(15 downto 0):= (others =>'0');
signal nrm_data_in: std_logic_vector(15 downto 0):= (others => '0');
signal nrm_wr_enable: std_logic := '0';
signal nrm_sum:integer :=0;
signal display_counter:integer :=0;
-------------------------------------------
--------------------------------------------------------
constant HACTIVE: integer:=639;
signal reg00,reg02,reg01,reg10,reg11,reg12,reg20,reg21,reg22: integer := 2;
signal pix00,pix01,pix02,pix10,pix11,pix12,pix20,pix21,pix22: integer := 0;
constant HFRONT: integer:=16;
constant HSYNC_PIXEL: integer:=96;
constant HBACK: integer:=48;
constant VACTIVE: integer:=479;
constant VFRONT: integer:=10;
constant VSYNC_PIXEL: integer:=2;
constant VBACK: integer:=33;
signal hcnt: integer:=0;
signal reset : std_logic :='0';
signal Vcnt: integer:=0;
signal Video_On: std_logic:='0';
--------------------------------------------------------
signal i,j:integer :=0;
signal grad_cycle:integer :=0;
signal rom_add: std_logic_vector(11 downto 0) := (others => '0');
signal ram_add: std_logic_vector(11 downto 0) := (others => '0');
signal data_rom: std_logic_vector(7 downto 0) := (others => '0');
signal sum: integer :=0;
signal wr_enable: std_logic := '1';
signal temp_max_val: integer := -65536;
signal temp_min_val: integer :=65536;
-------------------------------------------
component dist_mem_gen_0 ---- This ROM stores the values of filter
port(a: in std_logic_vector(3 downto 0);
    spo: out std_logic_vector(7 downto 0);
    clk: in std_logic);
end component;

component dist_mem_gen_4 ---- This ROM stores the values of input image
port(a: in std_logic_vector(11 downto 0);
    spo: out std_logic_vector(7 downto 0);
    clk: in std_logic
    );
end component;

component dist_mem_gen_2 ---- This RAM stores the values of pixels after applying 3x3 filter
port(a: in std_logic_vector(11 downto 0);
    d: in std_logic_vector(15 downto 0);
    clk : in std_logic;
    we: in std_logic;
    spo : out std_logic_vector(15 downto 0)
    );
end component;

component dist_mem_gen_3 ---- This RAM stores the values of pixels after Normalisation
port(a: in std_logic_vector(11 downto 0);
    d: in std_logic_vector(15 downto 0);
    clk : in std_logic;
    we: in std_logic;
    spo : out std_logic_vector(15 downto 0)
    );
end component;
-------------------------------------------------

begin
-------------------------------------------------------------

rom1: dist_mem_gen_0 port map(a=>ker_rom_add,clk=>clk_slow,spo=>data_rom_ker);

rom2: dist_mem_gen_4 port map(a => rom_add,clk=>clk_slow,spo => data_rom);

ram : dist_mem_gen_2 port map(a=>ram_add,clk=>clk_slow,we => wr_enable,d=>data_in,spo => output_pixel);

ram2: dist_mem_gen_3 port map(a=>nrm_ram_add,clk=>clk_slow,we => nrm_wr_enable,d=>nrm_data_in,spo => nrm_output_pixel);
--------------------------------------------------
  
----------Clock Divider----------------------------
process(clk)
variable helper: integer:=0;
    begin
        if(rising_edge(clk)) then
            case helper is
            when 0 =>
                clk_slow<='1';
                helper:=helper+1; 
            when 1 =>
                helper:=helper+1;
            when 2 =>
                clk_slow<='0';
                helper:=helper+1;
            when others =>
                helper :=0;
            end case;
        end if;
end process;
------------------------------------------------------------------------------
------------------------------------------------------------------------------


--------------------Filter Reading Process-----------------------------------------
process(clk_slow) 
begin
if(rising_edge(clk_slow)) then
    if(FSM_state =0) then
        case kernel_comp is
        when 0 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 1 => 
            kernel_comp<=kernel_comp+1;
        when 2 => 
            kernel_itr<=kernel_itr+1;
            reg00<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 3 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 4 => 
            kernel_comp<=kernel_comp+1;
        when 5 => 
            kernel_itr<=kernel_itr+1;
            reg01<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 6 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 7 => 
            kernel_comp<=kernel_comp+1;
        when 8 => 
            kernel_itr<=kernel_itr+1;
            reg02<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 9 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 10 => 
            kernel_comp<=kernel_comp+1;
        when 11 => 
            kernel_itr<=kernel_itr+1;
            reg10<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 12 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 13 => 
            kernel_comp<=kernel_comp+1;
        when 14 => 
            kernel_itr<=kernel_itr+1;
            reg11<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 15 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 16 => 
            kernel_comp<=kernel_comp+1;
        when 17 => 
            kernel_itr<=kernel_itr+1;
            reg12<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 18 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 19 => 
            kernel_comp<=kernel_comp+1;
        when 20 => 
            kernel_itr<=kernel_itr+1;
            reg20<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 21 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 22 => 
            kernel_comp<=kernel_comp+1;
        when 23 => 
            kernel_itr<=kernel_itr+1;
            reg21<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        when 24 => 
            ker_rom_add<=std_logic_vector(to_unsigned(kernel_itr,4));
            kernel_comp<=kernel_comp+1;
        when 25 => 
            kernel_comp<=kernel_comp+1;
        when others => 
            kernel_itr<=kernel_itr+1;
            reg22<=to_integer(signed(data_rom_ker));
            kernel_comp<=kernel_comp+1;
        end case;
    end if;
end if;
end process;
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------

-------------------------MAC Process------------------------------------------------------
process(clk_slow)

begin 
    
    if(rising_edge(clk_slow)) then 
    if(FSM_state =1) then
        
        if(i=0) then 
            if(j=0) then 
                case grad_cycle is
                when 0 => 
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 7 =>
                    grad_cycle<= grad_cycle+1;
                when 8 => 
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val < pix11 ) then 
                        temp_max_val <= pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 =>
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    j<=j+1;
                    grad_cycle<=0;
                end case;            
            elsif(j=63) then
                case grad_cycle is 
                when 0 => 
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 =>
                    pix00<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 7 => 
                    grad_cycle<= grad_cycle+1;
                when 8 =>
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    i<=i+1;
                    j<=0;
                    grad_cycle<=0;
                end case;
            else
                case grad_cycle is 
                when 0 => 
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 7 => 
                    grad_cycle<= grad_cycle+1;
                when 8 =>
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 =>
                    pix12<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 23 =>
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    j<=j+1;
                    grad_cycle<=0;
                end case;
            end if;
        elsif(i=63) then 
            if(j=0) then
                case grad_cycle is 
                when 0 => 
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 7 => 
                    grad_cycle<= grad_cycle+1;
                when 8 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=0;
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    grad_cycle<=grad_cycle+1 ;
                when 33 =>
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    j<=j+1;
                    grad_cycle<=0;
                end case;
            elsif(j=63) then 
                case grad_cycle is
                when 0 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 7 =>
                    grad_cycle<= grad_cycle+1;
                when 8 => 
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=0;
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    i<=64;
                    j<=0;  
                    grad_cycle<=0;
                end case;
            else
                case grad_cycle is 
                when 0 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 7 => 
                    grad_cycle<= grad_cycle+1;
                when 8 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 =>
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=0;
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    j<=j+1;
                    grad_cycle<=0;
                end case;
            end if;
        else 
            if(j=0) then 
                case grad_cycle is
                when 0 => 
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 7 => 
                    grad_cycle<= grad_cycle+1;
                when 8 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    j<=j+1;
                    grad_cycle<=0;
                end case;
            elsif(j=63) then 
                case grad_cycle is
                when 0 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 7 => 
                    grad_cycle<= grad_cycle+1;
                when 8 => 
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=0;
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    i<=i+1;
                    j<=0;
                    grad_cycle<=0;
                end case;
            else 
                case grad_cycle is
                when 0 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 1 => 
                    grad_cycle<= grad_cycle+1;
                when 2 => 
                    pix00<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 3 => 
                    grad_cycle<= grad_cycle+1;
                when 4 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 5 => 
                    grad_cycle<= grad_cycle+1;
                when 6 => 
                    pix01<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 7 => 
                    grad_cycle<= grad_cycle+1;
                when 8 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i-1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 9 => 
                    grad_cycle<= grad_cycle+1;
                when 10 => 
                    pix02<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 11 => 
                    grad_cycle<= grad_cycle+1;
                when 12 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 13 => 
                    grad_cycle<= grad_cycle+1;
                when 14 => 
                    pix10<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 15 => 
                    grad_cycle<= grad_cycle+1;
                when 16 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 17 => 
                    grad_cycle<= grad_cycle+1;
                when 18 => 
                    pix11<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 19 => 
                    if(temp_max_val<pix11) then 
                        temp_max_val<=pix11;
                    end if;
                    if(temp_min_val>pix11) then 
                        temp_min_val<=pix11;
                    end if;
                    grad_cycle<= grad_cycle+1;
                when 20 => 
                    rom_add<=std_logic_vector(to_unsigned(64*i+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 21 => 
                    grad_cycle<= grad_cycle+1;
                when 22 => 
                    pix12<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 23 => 
                    grad_cycle<= grad_cycle+1;
                when 24 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j-1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 25 => 
                    grad_cycle<= grad_cycle+1;
                when 26 => 
                    pix20<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 27 => 
                    grad_cycle<= grad_cycle+1;
                when 28 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j,12));
                    grad_cycle<=grad_cycle+1 ;
                when 29 => 
                    grad_cycle<= grad_cycle+1;
                when 30 => 
                    pix21<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1;
                when 31 => 
                    grad_cycle<= grad_cycle+1;
                when 32 => 
                    rom_add<=std_logic_vector(to_unsigned(64*(i+1)+j+1,12));
                    grad_cycle<=grad_cycle+1 ;
                when 33 => 
                    grad_cycle<= grad_cycle+1;
                when 34 => 
                    pix22<=to_integer(unsigned(data_rom));
                    grad_cycle<=grad_cycle+1 ;
                when 35 => 
                    grad_cycle<= grad_cycle+1;
                when 36 => 
                    grad_cycle<=grad_cycle+1 ;
                when 37 => 
                    grad_cycle<= grad_cycle+1;
                when 38 => 
                    sum<=reg00*pix00+reg01*pix01+reg02*pix02+reg10*pix10+reg11*pix11+reg12*pix12+reg20*pix20+reg21*pix21+reg22*pix22;
                    grad_cycle<=grad_cycle+1;
                when 39 => 
                    if(final_max_val<sum) then 
                        final_max_val<=sum;
                    end if;
                    if(final_min_val>sum) then 
                        final_min_val<=sum;
                    end if;
                    data_in<=std_logic_vector(to_signed(sum, 16));
                    grad_cycle<=grad_cycle+1;
                when others => 
                    j<=j+1;
                    grad_cycle<=0;
                end case;        
            end if;
        end if;
end if;
    end if;
end process;
--------------------------------------------------------------------------------
-------------------------------------------------------------------------------

process (clk_slow)

begin

if(rising_edge(clk_slow)) then

    if(FSM_state =2) then
        case nrm_comp is 
        when 0 =>
            nrm_comp<=1;
        when 1 =>
            nrm_comp<=2;
        when 2 =>
            nrm_comp<=3;
            nrm_sum<=to_integer(signed(output_pixel));
        when 3 =>
            nrm_comp<=4;
        when 4 =>
            nrm_comp<=5;
        when 5 =>
            nrm_sum<=(nrm_sum-final_min_val)*(255);
            nrm_comp<=6;
        when 6 =>
            nrm_comp<=7;
        when 7 =>
            nrm_sum<=(nrm_sum)/(final_max_val-final_min_val);
            nrm_comp<=8;
        when 8 =>
            nrm_comp<=9;
        when 9 =>
            nrm_comp <= 10;
        when 10 =>
            nrm_data_in<=std_logic_vector(to_signed(nrm_sum, 16));
            nrm_comp<=11;
        when 11 =>
            if(jtr_nrml=63) then
                jtr_nrml<=0;
                itr_nrml<=itr_nrml+1;
            else
                jtr_nrml<=jtr_nrml+1;
            end if;
            nrm_comp<=12;
        when 12 =>
            nrm_comp<=13;
        when others =>
            nrm_comp<=0;
        end case;
    end if;
end if;

end process;
--------------------------------------------------------------------------------
--------------------------------------------------------------------------------
--------------------Main VGA Controller--------------------------------
process(clk_slow, reset , hcnt, Vcnt, Video_On)
begin
    if(rising_edge(clk_slow)) then
        if (Video_On='1') then
            if((hcnt>=0 and hcnt<=64 ) and (Vcnt>=0 and Vcnt<=64 )) then
                r <= nrm_output_pixel(7 downto 4);
                g <= nrm_output_pixel(7 downto 4);
                b <= nrm_output_pixel(7 downto 4);
                if(hcnt=64 and Vcnt=64) then
                    display_counter<=0;
                else
                    display_counter<=display_counter+1;
                end if;
            else
                r<= (others => '0');
                g<= (others => '0');
                b<= (others => '0');
            end if;
        else
            r<= (others => '0');
            g<= (others => '0');
            b<= (others => '0');
        end if;
    end if;
end process;
-----------------------------------------------------------------
------------------------------------------------------------------
------------------This Process is FSM----------------------------------
process(clk_slow,i,j,itr_nrml,jtr_nrml,FSM_state)
begin
    if(rising_edge(clk_slow)) then
        if(FSM_state = 1) then
            ram_add<=std_logic_vector(TO_UNSIGNED(64*i+j, 12));
            wr_enable<='1';
        end if;
        if(FSM_state =2) then
            ram_add<=std_logic_vector(TO_UNSIGNED(64*itr_nrml+jtr_nrml, 12));
            nrm_ram_add<=std_logic_vector(TO_UNSIGNED(64*itr_nrml+jtr_nrml,12));
            wr_enable<='0';
            nrm_wr_enable <='1';
        end if;
        if(FSM_state =3) then
            nrm_ram_add<=std_logic_vector(TO_UNSIGNED(display_counter, 12));         
            nrm_wr_enable <='0';
        end if;
    end if;
end process;
-------------------------------------------------------------------------------



--------------------------Vertical Synchronization--------------------------------
process(clk_slow,reset,Vcnt)
begin
    if(reset='1') then
        vsync <= '0';
    elsif(rising_edge(clk_slow)) then
        if(Vcnt<=(VACTIVE+VFRONT) or Vcnt>(VACTIVE+VFRONT+VSYNC_PIXEL)) then
            vsync<='1';
        else
            vsync<='0';
        end if;
    end if;
end process;
-----------------------------------------------------------------------------------------





--------------------------Vertical Counter ------------------------------------
process(clk_slow,reset,hcnt)
begin
    if(reset='1') then
        Vcnt<=0;
    elsif(rising_edge(clk_slow)) then
        if(hcnt= HACTIVE+HFRONT+HSYNC_PIXEL+HBACK) then
            if(Vcnt= VACTIVE+VFRONT+VSYNC_PIXEL+VBACK) then
               Vcnt<=0;
            else
               Vcnt<=Vcnt+1;
            end if;
          end if;
       end if;
end process;
------------------------------------------------------------------------

------------------Horizontal Synchronization----------------------------
process(clk_slow,reset,hcnt)
begin
    if(reset='1') then
        hsync <= '0';
    elsif(rising_edge(clk_slow)) then
        if(hcnt<=(HACTIVE+HFRONT) or hcnt>(HACTIVE+HFRONT+HSYNC_PIXEL)) then
            hsync<='1';
        else
            hsync<='0';
        end if;
    end if;
end process;
-------------------------------------------------------------------------------

----------------------------Video_On-----------------------------
process(clk_slow,reset,hcnt,Vcnt)
begin
    if(rising_edge(clk_slow)) then
        if(hcnt<=HACTIVE and Vcnt<=VACTIVE) then
            Video_On<='1';
        else
            Video_On<='0';
        end if;
    end if;
end process;
--------------------------------------------------------------------

------------------------------Horizontal Counter ----------------------------
process(clk_slow,reset)
begin
    if(reset='1') then
        hcnt<=0;
    elsif(rising_edge(clk_slow)) then
        if(hcnt= HACTIVE+HFRONT+HSYNC_PIXEL+HBACK) then
            hcnt<=0;
        else
            hcnt<=hcnt+1;
        end if;
    end if;
end process;
------------------------------------------------------------------------------
------------------This Process is Transitions of States----------------------------------
process(clk_slow,i,j,itr_nrml,jtr_nrml)
begin
    if(rising_edge(clk_slow)) then
        if(kernel_comp>26) then
            FSM_state <= 1;
        end if;
        if(kernel_comp>26 and i=64 and itr_nrml<64) then
            FSM_state <= 2;
        end if;
        if(itr_nrml=64) then
            FSM_state <= 3;
        end if;
    end if;
end process;
-------------------------------------------------------------------------------

end Behavioral;