import cocotb
from cocotb.triggers import RisingEdge, Timer
from cocotb_tb.env.clock_reset import setup_clock, reset_dut
from backends.cocotb_backend import CocotbBackend


@cocotb.test()
async def run_system(dut):
 # === DRIVE ALL INPUTS TO KNOWN VALUES ===
    # Encoder inputs
    dut.enc_a.value = 0
    dut.enc_b.value = 0
    
    # UART inputs
    dut.uart_rx.value = 1      # idle high
    dut.uart_addr.value = 0
    dut.uart_data_in.value = 0
    dut.uart_wr.value = 0
    dut.uart_rd.value = 0
    
    # SPI inputs
    dut.spi_miso.value = 0
    
    # Feedback signals (these are driven by your plant model later,
    # but must be initialized to avoid 'U')
    dut.speed_fb.value = 0
    dut.sim_current_ma.value = 0


    await setup_clock(dut)
    await reset_dut(dut)
    
    # =========================
    # DEBUG: Check constants immediately after reset
    # =========================
    await Timer(100, units='ns')
    
    cocotb.log.error("=" * 80)
    cocotb.log.error("DEBUGGING METAVALUES - CONSTANTS CHECK")
    cocotb.log.error("=" * 80)
    
    # Check the ALPHA constants
    try:
        alpha_speed = dut.ALPHA_SPEED_RAW.value
        alpha_ref = dut.ALPHA_REF_RAW.value
        cocotb.log.info(f"ALPHA_SPEED_RAW = {alpha_speed} (bits: {bin(alpha_speed)})")
        cocotb.log.info(f"ALPHA_REF_RAW = {alpha_ref} (bits: {bin(alpha_ref)})")
        
        if hasattr(alpha_speed, "signed_integer"):
            cocotb.log.info(f"  As signed: {alpha_speed.signed_integer}")
            cocotb.log.info(f"  As unsigned: {int(alpha_speed)}")
    except Exception as e:
        cocotb.log.error(f"Could not read ALPHA constants: {e}")
    
    # =========================
    # CHECK ALL TOP-LEVEL PORTS FOR METAVALUES
    # =========================
    await Timer(100, units='ns')
    
    cocotb.log.error("\n" + "=" * 80)
    cocotb.log.error("CHECKING ALL TOP-LEVEL PORTS")
    cocotb.log.error("=" * 80)
    
    # Get all signals in the DUT
    all_signals = []
    for name in dir(dut):
        if not name.startswith('_') and hasattr(dut, name):
            try:
                attr = getattr(dut, name)
                if hasattr(attr, 'value'):
                    all_signals.append(name)
            except:
                pass
    
    metavalue_signals = []
    for sig_name in sorted(all_signals):
        try:
            sig = getattr(dut, sig_name)
            val = sig.value
            val_str = str(val)
            
            # Check for metavalues
            if 'U' in val_str or 'X' in val_str or 'Z' in val_str or 'W' in val_str:
                metavalue_signals.append((sig_name, val_str))
                cocotb.log.error(f"🔴 METAVALUE DETECTED: {sig_name} = {val_str}")
            else:
                # Try to get numeric value for non-metavalues
                try:
                    if hasattr(val, 'integer'):
                        cocotb.log.info(f"✅ {sig_name:30} = {val_str:10} (int: {val.integer})")
                    else:
                        cocotb.log.info(f"✅ {sig_name:30} = {val_str}")
                except:
                    cocotb.log.info(f"✅ {sig_name:30} = {val_str}")
        except Exception as e:
            cocotb.log.warning(f"Could not read {sig_name}: {e}")
    
    if metavalue_signals:
        cocotb.log.error("\n" + "=" * 80)
        cocotb.log.error(f"FOUND {len(metavalue_signals)} SIGNALS WITH METAVALUES:")
        for name, val in metavalue_signals:
            cocotb.log.error(f"  • {name} = {val}")
        cocotb.log.error("=" * 80)
    else:
        cocotb.log.info("\n✅ No metavalues found on top-level ports!")
    
    # =========================
    # CHECK INTERNAL SIGNALS (if accessible)
    # =========================
    cocotb.log.error("\n" + "=" * 80)
    cocotb.log.error("CHECKING COMMON INTERNAL SIGNALS")
    cocotb.log.error("=" * 80)
    
    internal_paths = [
        "dut.top",  # Adjust based on your hierarchy
        "dut.u_pi_controller",
        "dut.u_pwm_generator",
        "dut.u_encoder_interface",
        "dut.u_adc_interface"
    ]
    
    internal_signals = [
        "reset_n", "enable", "integrator", "previous_error",
        "pwm_duty", "speed_measured", "current_measured"
    ]
    
    for path in internal_paths:
        for sig_name in internal_signals:
            try:
                # Try different access patterns
                full_path = f"{path}.{sig_name}"
                parts = full_path.split('.')
                obj = dut
                for part in parts:
                    obj = getattr(obj, part)
                
                val = obj.value
                val_str = str(val)
                if 'U' in val_str or 'X' in val_str:
                    cocotb.log.error(f"🔴 INTERNAL METAVALUE: {full_path} = {val_str}")
                else:
                    cocotb.log.info(f"✅ {full_path:40} = {val_str}")
            except:
                pass  # Signal doesn't exist or can't be accessed
    
    # =========================
    # CONTINUOUS MONITORING DURING SIMULATION
    # =========================
    cocotb.log.error("\n" + "=" * 80)
    cocotb.log.error("STARTING MAIN SIMULATION WITH METAVALUE MONITORING")
    cocotb.log.error("=" * 80)
    
    # =========================
    # Backend (FIXED)
    # =========================
    backend = CocotbBackend(dut)
    backend.start_stream()

    # =========================
    # Plant state
    # =========================
    i_plant = 0.0
    w_plant = 0.0

    R, L, J, B, Kt, Ke = 2.5, 0.0005, 0.0001, 0.0001, 0.05, 0.05
    DT = 10e-9

    CURRENT_SCALE = 4096
    SPEED_SCALE = 128
    VOLTAGE_SCALE = 1024

    dut.speed_fb.value = 0
    dut.sim_current_ma.value = 0

    SAMPLE_PERIOD = 100000
    SETPOINT_TIME = 20000

    counter = 0
    last_metavalue_warning = 0

    for cycle in range(20000000):

        await RisingEdge(dut.clk)

        # =========================
        # MONITOR FOR NEW METAVALUES EVERY 10000 CYCLES
        # =========================
        if cycle % 10000 == 0 and cycle > 0:
            try:
                # Check v_cmd_test specifically
                v_cmd_val = dut.v_cmd_test.value
                if not v_cmd_val.is_resolvable or 'U' in str(v_cmd_val) or 'X' in str(v_cmd_val):
                    cocotb.log.error(f"🔴 cycle={cycle}: v_cmd_test has metavalue: {v_cmd_val}")
                
                # Check feedback signals
                speed_fb_val = dut.speed_fb.value
                if not speed_fb_val.is_resolvable or 'U' in str(speed_fb_val) or 'X' in str(speed_fb_val):
                    cocotb.log.error(f"🔴 cycle={cycle}: speed_fb has metavalue: {speed_fb_val}")
                
                current_val = dut.sim_current_ma.value
                if not current_val.is_resolvable or 'U' in str(current_val) or 'X' in str(current_val):
                    cocotb.log.error(f"🔴 cycle={cycle}: sim_current_ma has metavalue: {current_val}")
            except:
                pass

        # =========================
        # SETPOINT INJECTION
        # =========================
        if cycle == SETPOINT_TIME:
            dut.uart_addr.value = 0x00
            dut.uart_data_in.value = int(50 * SPEED_SCALE)    # 6400 decimal
            dut.uart_wr.value = 1
            cocotb.log.info(f"🎯 Setpoint injected at cycle {cycle}")
        else:
             dut.uart_wr.value = 0
        # =========================
        # READ DUT OUTPUT (SAFE)
        # =========================
        if not dut.v_cmd_test.value.is_resolvable:
            if cycle - last_metavalue_warning > 1000:
                cocotb.log.warning(f"cycle={cycle}: v_cmd_test not resolvable (metavalue) - skipping")
                last_metavalue_warning = cycle
            continue
        v_cmd_raw = dut.v_cmd_test.value.signed_integer
        v_cmd = v_cmd_raw / VOLTAGE_SCALE

        # =========================
        # PLANT MODEL
        # =========================
        di_dt = (v_cmd - R * i_plant - Ke * w_plant) / L
        dw_dt = (Kt * i_plant - B * w_plant) / J

        i_plant += di_dt * DT
        w_plant += dw_dt * DT

        # =========================
        # FEEDBACK TO DUT
        # =========================

        # Clamp current to 16-bit signed range
        current_raw = int(i_plant * CURRENT_SCALE)
        if current_raw > 32767:
            current_raw = 32767
        elif current_raw < -32768:
            current_raw = -32768
        dut.sim_current_ma.value = current_raw
        dut.speed_fb.value = int(w_plant * SPEED_SCALE)
        # =========================
        # LOGGING       
        # =========================
        counter += 1
  
        if counter >= SAMPLE_PERIOD:
            counter = 0

            backend.sample()
            data = backend.get_data()

            cocotb.log.info(
            f"t={cycle} | speed={w_plant:.2f} | current={i_plant:.2f} | v_cmd_raw={v_cmd_raw}"
            )

            cocotb.log.info(f"speed_ref_raw = {dut.speed_setpoint.value}")
            cocotb.log.info(f"speed_fb_raw = {int(w_plant * SPEED_SCALE)}")
    # FINAL METAVALUE CHECK
    # =========================
    await Timer(100, units='ns')

    final_metavalues = []
    for sig_name in all_signals:
        try:
            sig = getattr(dut, sig_name)
            val_str = str(sig.value)
            if 'U' in val_str or 'X' in val_str:
                final_metavalues.append(sig_name)
                cocotb.log.error(f"🔴 {sig_name} = {val_str}")
        except:
            pass
    
    if final_metavalues:
        cocotb.log.error(f"\n❌ TEST COMPLETED BUT {len(final_metavalues)} SIGNALS STILL HAVE METAVALUES")
        cocotb.log.error(f"   Signals: {', '.join(final_metavalues)}")
    else:
        cocotb.log.info("\n✅ No metavalues remain at end of test")
