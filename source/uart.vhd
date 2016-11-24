-------------------------------------------------------------------------------
-- UART
-- Implements a universal asynchronous receiver transmitter
-------------------------------------------------------------------------------
-- clock
--      Input clock, must match frequency value given on clock_frequency
--      generic input.
-- reset
--      Synchronous reset.  
-- data_stream_in
--      Input data bus for bytes to transmit.
-- data_stream_in_stb
--      Input strobe to qualify the input data bus.
-- data_stream_in_ack
--      Output acknowledge to indicate the UART has begun sending the byte
--      provided on the data_stream_in port.
-- data_stream_out
--      Data output port for received bytes.
-- data_stream_out_stb
--      Output strobe to qualify the received byte. Will be valid for one clock
--      cycle only. 
-- tx
--      Serial transmit.
-- rx
--      Serial receive
-------------------------------------------------------------------------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
-- Math/log2,ceil required to get the number of bits for our counter
use ieee.math_real.log2;
use ieee.math_real.ceil;


entity UART is
    Generic (
            -- Baudrate in bps
            -- The baudrate must satisify the following condition:
            -- BAUD_DIVIDER := truncate(CLOCK_FREQUENCY/BAUD_RATE)
            -- remainder(BAUD_DIVIDER/16) == 0
            -- Why: 16 times oversampling on the receiver side
            -- Also take care that the remainder(CLOCK_FREQUENCY/BAUD_RATE) is
            -- small because this determines the UART baud rate error
            -- See constant c_oversample_divider_val for more information
            BAUD_RATE           : positive;
            -- Input Clock frequency in Hz
            CLOCK_FREQUENCY     : positive
        );
    Port (
            -- System Clock
            CLOCK               :   in      std_logic;
            -- High-Active Asynchronous Reset
            RESET               :   in      std_logic;
            -- The input data: 8 bit - this is the UART sender
            -- Provide data on DATA_STREAM_IN and set STB to high
            -- Keep the data stable until ACK is set to high which shows that
            -- the data is copied into the internal buffer. Then you should
            -- revoke STB and you can change IN as you want.
            DATA_STREAM_IN      :   in      std_logic_vector(7 downto 0);
            DATA_STREAM_IN_STB  :   in      std_logic;
            DATA_STREAM_IN_ACK  :   out     std_logic := '0';
            -- The output data: 8 bit - this is the UART receiver
            -- Data is only valid during the time the STB is high
            -- Acknowledge the data with a pulse on ACK, which is confirmed by
            -- revoking STB.
            -- When the following start bit is received the data becomes
            -- invalid and the STB is revoked. So take care about fetching the
            -- data early enough, or install your own FIFO buffer
            DATA_STREAM_OUT     :   out     std_logic_vector(7 downto 0);
            DATA_STREAM_OUT_STB :   out     std_logic;
            DATA_STREAM_OUT_ACK :   in      std_logic;
            TX                  :   out     std_logic;
            RX                  :   in      std_logic  -- Async Receive
         );
end UART;

architecture RTL of UART is
    ----------------------------------------------------------------------------
    -- BAUD Generation
    ----------------------------------------------------------------------------
    -- First create the divider for the 16 times oversampled baud rate,
    -- the baud rate then is derived by dividing by 16.
    -- Thats why the 16 times oversampling clock must be derived without any reminder left
    -- from the baud rate, to not disturb the resulting bit rate
    -- You need to take care about this when selecting baud and clock frequency
    -- Substract one, otherwise the reloading step is counted twice
    constant c_oversample_divider_steps : natural := natural(CLOCK_FREQUENCY / (16*BAUD_RATE))-1;
    -- And also how many bits do we need?
    constant c_oversample_divider_bits : natural := natural(ceil(log2(real(c_oversample_divider_steps))));
    -- And this is the counter type we use
    subtype oversample_baud_counter_type is unsigned(c_oversample_divider_bits-1 downto 0);
    -- Please only use this final value
    constant c_oversample_divider_val : oversample_baud_counter_type  := to_unsigned(c_oversample_divider_steps, c_oversample_divider_bits);
    -- Datatype for the rx and tx counter type, must accomodate for the 8bit positions
    subtype uart_rxtx_count_type is unsigned(2 downto 0);
    constant c_uart_rxtx_count_reset : uart_rxtx_count_type  := "000";  -- Reset value: 0

    signal oversample_baud_counter    : oversample_baud_counter_type := c_oversample_divider_val;
    -- Tick created every counter reset
    signal oversample_baud_tick       : std_ulogic := '0';
    -- At this moment we sample the incoming signal
    signal uart_rx_sample_tick        :   std_ulogic := '0';
    -- The baud rate itself is the oversampling tick divided by 16
    subtype baud_counter_type is unsigned(3 downto 0);
    signal baud_counter  :   baud_counter_type := ( others => '1');
    signal baud_tick     :   std_ulogic := '0';

    ----------------------------------------------------------------------------
    -- Transmitter Signals
    ----------------------------------------------------------------------------
    type    uart_tx_states is ( idle,
                                wait_for_tick,
                                send_start_bit,
                                transmit_data,
                                send_stop_bit);

    signal  uart_tx_state       : uart_tx_states := idle;

    signal  uart_tx_data_block  : std_logic_vector(7 downto 0) := (others => '0');
    signal  uart_tx_data        : std_logic := '1';
    signal  uart_tx_count       : uart_rxtx_count_type := c_uart_rxtx_count_reset; -- 8 states, stored in 3 bits
    signal  uart_rx_data_in_ack : std_logic := '0';
    ----------------------------------------------------------------------------
    -- Receiver Signals
    ----------------------------------------------------------------------------
    type    uart_rx_states is ( rx_wait_start_synchronise,  -- Wait and deliver data
                                rx_get_start_bit,  -- We are reading the start bit
                                rx_get_data,
                                rx_get_stop_bit);

architecture rtl of uart is
    ---------------------------------------------------------------------------
    -- Baud generation constants
    ---------------------------------------------------------------------------
    constant c_tx_div       : integer := clock_frequency / baud;
    constant c_rx_div       : integer := clock_frequency / (baud * 16);
    constant c_tx_div_width : integer 
        := integer(log2(real(c_tx_div))) + 1;   
    constant c_rx_div_width : integer 
        := integer(log2(real(c_rx_div))) + 1;
    ---------------------------------------------------------------------------
    -- Baud generation signals
    ---------------------------------------------------------------------------
    signal tx_baud_counter : unsigned(c_tx_div_width - 1 downto 0) 
        := (others => '0');   
    signal tx_baud_tick : std_logic := '0';
    signal rx_baud_counter : unsigned(c_rx_div_width - 1 downto 0) 
        := (others => '0');   
    signal rx_baud_tick : std_logic := '0';
    ---------------------------------------------------------------------------
    -- Transmitter signals
    ---------------------------------------------------------------------------
    type uart_tx_states is ( 
        tx_send_start_bit,
        tx_send_data,
        tx_send_stop_bit
    );             
    signal uart_tx_state : uart_tx_states := tx_send_start_bit;
    signal uart_tx_data_vec : std_logic_vector(7 downto 0) := (others => '0');
    signal uart_tx_data : std_logic := '1';
    signal uart_tx_count : unsigned(2 downto 0) := (others => '0');
    signal uart_rx_data_in_ack : std_logic := '0';
    ---------------------------------------------------------------------------
    -- Receiver signals
    ---------------------------------------------------------------------------
    type uart_rx_states is ( 
        rx_get_start_bit, 
        rx_get_data, 
        rx_get_stop_bit
    );            
    signal uart_rx_state : uart_rx_states := rx_get_start_bit;
    signal uart_rx_bit : std_logic := '1';
    signal uart_rx_data_vec : std_logic_vector(7 downto 0) := (others => '0');
    signal uart_rx_data_sr : std_logic_vector(1 downto 0) := (others => '1');
    signal uart_rx_filter : unsigned(1 downto 0) := (others => '1');
    signal uart_rx_count : unsigned(2 downto 0) := (others => '0');
    signal uart_rx_data_out_stb : std_logic := '0';
    signal uart_rx_bit_spacing : unsigned (3 downto 0) := (others => '0');
    signal uart_rx_bit_tick : std_logic := '0';
begin

    ----------------------------------------------------------------------------
    -- Transmitter Part: Sending Data
    ----------------------------------------------------------------------------
    TX                  <= uart_tx_data;

    -- The input clock is CLOCK_FREQUENCY
    -- For example it's set to 100MHz, then needs to be divided down to the
    -- rate dictated by the BAUD_RATE. For example, if 115200 baud is selected
    -- (115200 baud = 115200 bps - 115.2kbps) a tick must be generated once
    -- every 1/115200
    -- As explained above we use a two-step approach, so we just scale down
    -- here the 16-times oversampled RX clock again
    -- Use a down-counter to have a simple test for zero
    -- Thats the counter part
    TX_CLOCK_DIVIDER   : process (CLOCK, RESET)
    begin
        if rising_edge (clock) then
            if reset = '1' then
                rx_baud_counter <= (others => '0');
                rx_baud_tick <= '0';    
            else
                if rx_baud_counter = c_rx_div then
                    rx_baud_counter <= (others => '0');
                    rx_baud_tick <= '1';
                else
                    rx_baud_counter <= rx_baud_counter + 1;
                    rx_baud_tick <= '0';
                end if;
              end if;
      end if;
    end process TX_CLOCK_DIVIDER;

    -- And thats the baud tick, which is of course only one clock long
    -- So both counters should be Zero
    TX_TICK: baud_tick <= '0' when RESET = '1' else
                          '1' when oversample_baud_tick = '1' and baud_counter = 0 else
                          '0';

    -- Get data from DATA_STREAM_IN and send it one bit at a time
    -- upon each BAUD tick. LSB first.
    -- Wait 1 tick, Send Start Bit (0), Send Data 0-7, Send Stop Bit (1)
    UART_SEND_DATA: process(CLOCK, RESET)
    begin
      if RESET = '1' then
                uart_tx_data            <= '1';
                uart_tx_data_block      <= (others => '0');
                uart_tx_count           <= c_uart_rxtx_count_reset;
                uart_tx_state           <= idle;
                uart_rx_data_in_ack     <= '0';
      elsif rising_edge(CLOCK) then
                uart_rx_data_in_ack     <= '0';
                case uart_tx_state is
                    when idle =>
                        if DATA_STREAM_IN_STB = '1' then
                            uart_tx_data_block  <= DATA_STREAM_IN;
                            uart_rx_data_in_ack <= '1';
                            uart_tx_state       <= wait_for_tick;
                        end if;
                    when wait_for_tick =>
                        if baud_tick = '1' then
                            uart_tx_state   <= send_start_bit;
                        end if;
                    when send_start_bit =>
                        if baud_tick = '1' then
                            uart_tx_data    <= '0';
                            uart_tx_state   <= transmit_data;
                        end if;
                    when transmit_data =>
                        if baud_tick = '1' then
                          -- Send next bit
                          uart_tx_data    <=  uart_tx_data_block(0);
                          -- Shift for next transmit bit, filling with don't care
                          -- Xilinx ISE does not know srl? So just build it ourself, hehe
			  -- uart_tx_data_block <=  uart_tx_data_block srl 1;
                          uart_tx_data_block <= shift_right_by_one(uart_tx_data_block, '-');
                          if uart_tx_count = 7 then  -- binary 111
                            -- We're done, move to next state
                            uart_tx_state   <= send_stop_bit;
                          else
                            -- Stay in current state
                            uart_tx_state   <= transmit_data;
                          end if;
                          -- Always increment here, will go to zero if we're out
                          uart_tx_count   <= uart_tx_count + 1;
                        end if;
                    when send_stop_bit =>
                        if baud_tick = '1' then
                            uart_tx_data <= '1';
                            uart_tx_state <= idle;
                        end if;
                    when others =>
                        uart_tx_data <= '1';
                        uart_tx_state <= idle;
                end case;
      end if;
    end process UART_SEND_DATA;

    ----------------------------------------------------------------------------
    -- Receiver Part: Getting Data
    ----------------------------------------------------------------------------
    DATA_STREAM_IN_ACK  <= uart_rx_data_in_ack;
    DATA_STREAM_OUT     <= uart_rx_data_block;
    DATA_STREAM_OUT_STB <= uart_rx_data_out_stb;

    -- The RX clock divider uses the 16 times oversampled clock, which we
    -- create here from the input clock
    -- Use a down-counter to have a simple test for zero
    -- Thats for the counter and tick creation part
    RX_CLOCK_DIVIDER: process (CLOCK, RESET)
    begin
      if RESET = '1' then
        oversample_baud_counter     <= c_oversample_divider_val;
        oversample_baud_tick        <= '0';
      elsif rising_edge (CLOCK) then
                if oversample_baud_counter = 0 then
                    oversample_baud_counter <= c_oversample_divider_val;
                    oversample_baud_tick    <= '1';
                else
                    oversample_baud_counter <= oversample_baud_counter - 1;
                    oversample_baud_tick    <= '0';
                end if;
      end if;
    end process RX_CLOCK_DIVIDER;

    -- We create the sample time by syncing the oversampled tick (BAUD * 16)
    -- to the received start bit by comparing then vs. the stored receive sync value
    -- It's only one clock tick active
    RX_SAMPLE: uart_rx_sample_tick <= '0' when RESET = '1' else
                                      '1' when oversample_baud_tick = '1' and uart_rx_sync_clock = baud_counter else
                                      '0';

    -- Synchronise RXD and Filter to suppress spikes with a 2 bit counter
    -- This is done with the 16-times oversampled clock
    -- Take care, every time the receive clock is resynchronized to the next
    -- start bit we can have somewhat of a jump here. But thats no problem
    -- because the jump (in case it occur) is still synchronous. And we save us
    -- another counter :-)
    RXD_SYNC_FILTER: process(CLOCK, RESET)
    begin
        if rising_edge(clock) then
            if reset = '1' then
                uart_rx_filter <= (others => '1');
                uart_rx_bit <= '1';
            else
                if rx_baud_tick = '1' then
                    -- filter rxd.
                    if uart_rx_data_sr(1) = '1' and uart_rx_filter < 3 then
                        uart_rx_filter <= uart_rx_filter + 1;
                    elsif uart_rx_data_sr(1) = '0' and uart_rx_filter > 0 then
                        uart_rx_filter <= uart_rx_filter - 1;
                    end if;
                    -- set the rx bit.
                    if uart_rx_filter = 3 then
                        uart_rx_bit <= '1';
                    elsif uart_rx_filter = 0 then
                        uart_rx_bit <= '0';
                    end if;
                end if;
      end if;
    end process RXD_SYNC_FILTER;

    UART_RECEIVE_DATA: process(CLOCK, RESET)
    begin
        if rising_edge(clock) then
            if reset = '1' then
                uart_rx_state <= rx_get_start_bit;
                uart_rx_data_vec <= (others => '0');
                uart_rx_count <= (others => '0');
                uart_rx_data_out_stb <= '0';
            else
                uart_rx_data_out_stb <= '0';
                case uart_rx_state is
                    when rx_get_start_bit =>
                        if rx_baud_tick = '1' and uart_rx_bit = '0' then
                            uart_rx_state <= rx_get_data;
                        end if;
                    when rx_get_data =>
                        if uart_rx_bit_tick = '1' then
                            uart_rx_data_vec(uart_rx_data_vec'high) 
                                <= uart_rx_bit;
                            uart_rx_data_vec(
                                uart_rx_data_vec'high-1 downto 0
                            ) <= uart_rx_data_vec(
                                uart_rx_data_vec'high downto 1
                            );
                            if uart_rx_count < 7 then
                                uart_rx_count   <= uart_rx_count + 1;
                            else
                                uart_rx_count <= (others => '0');
                                uart_rx_state <= rx_get_stop_bit;
                            end if;
                        end if;
                    when rx_get_stop_bit =>
                        if uart_rx_bit_tick = '1' then
                            if uart_rx_bit = '1' then
                                uart_rx_state <= rx_get_start_bit;
                                uart_rx_data_out_stb <= '1';
                            end if;
                        end if;                            
                    when others =>
                        uart_rx_state <= rx_get_start_bit;
                end case;
            end if;
        end if;
    end process uart_receive_data;
    ---------------------------------------------------------------------------
    -- TX_CLOCK_DIVIDER
    -- Generate baud ticks at the required rate based on the input clock
    -- frequency and baud rate
    ---------------------------------------------------------------------------
    tx_clock_divider : process (clock)
    begin
        if rising_edge (clock) then
            if reset = '1' then
                tx_baud_counter <= (others => '0');
                tx_baud_tick <= '0';    
            else
                if tx_baud_counter = c_tx_div then
                    tx_baud_counter <= (others => '0');
                    tx_baud_tick <= '1';
                else
                    tx_baud_counter <= tx_baud_counter + 1;
                    tx_baud_tick <= '0';
                end if;
            end if;
        end if;
    end process tx_clock_divider;
    ---------------------------------------------------------------------------
    -- UART_SEND_DATA 
    -- Get data from data_stream_in and send it one bit at a time upon each 
    -- baud tick. Send data lsb first.
    -- wait 1 tick, send start bit (0), send data 0-7, send stop bit (1)
    ---------------------------------------------------------------------------
    uart_send_data : process(clock)
    begin
        if rising_edge(clock) then
            if reset = '1' then
                uart_tx_data <= '1';
                uart_tx_data_vec <= (others => '0');
                uart_tx_count <= (others => '0');
                uart_tx_state <= tx_send_start_bit;
                uart_rx_data_in_ack <= '0';
            else
                uart_rx_data_in_ack <= '0';
                case uart_tx_state is
                    when tx_send_start_bit =>
                        if tx_baud_tick = '1' and data_stream_in_stb = '1' then
                            uart_tx_data  <= '0';
                            uart_tx_state <= tx_send_data;
                            uart_tx_count <= (others => '0');
                            uart_rx_data_in_ack <= '1';
                            uart_tx_data_vec <= data_stream_in;
                        end if;
                    when tx_send_data =>
                        if tx_baud_tick = '1' then
                            uart_tx_data <= uart_tx_data_vec(0);
                            uart_tx_data_vec(
                                uart_tx_data_vec'high-1 downto 0
                            ) <= uart_tx_data_vec(
                                uart_tx_data_vec'high downto 1
                            );
                            if uart_tx_count < 7 then
                                uart_tx_count <= uart_tx_count + 1;
                            else
                                uart_tx_count <= (others => '0');
                                uart_tx_state <= tx_send_stop_bit;
                            end if;
                        end if;
                    when tx_send_stop_bit =>
                        if tx_baud_tick = '1' then
                            uart_tx_data <= '1';
                            uart_tx_state <= tx_send_start_bit;
                        end if;
                    when others =>
                        uart_tx_data <= '1';
                        uart_tx_state <= tx_send_start_bit;
                end case;
            end if;
        end if;
    end process uart_send_data;    
end rtl;
