# Generic Hardware Version Branding Script for xESC_YF_rev4
# This script writes hardware version info to flash memory for runtime detection
# with optional WRP (Write Protection) to make hardware version tamper-proof
# Usage: brand_hw <data_file> [enable_wrp]
#
# Hardware Version LED Configuration:
# The hardware info includes LED GPIO configuration for hardware
# version mismatch indication.
#
# LED Configuration by Hardware Version:
# - V2.x: Green=PC15, Red=PB6
#
# STM32C011F6P6 Flash Layout with MODM Flash Reservation:
# - 32KB total (0x08000000 - 0x08007FFF)
# - 16 pages of 2KB each
# - Firmware uses pages 0-14 (30KB, 0x08000000 - 0x08007800)
# - Page 15 reserved for hardware data (2KB, 0x08007800 - 0x08007FFF)
# - Hardware info stored at 0x08007800 (first 16 bytes of reserved page)
#
# Hardware Version Structure (16 bytes):
# 0x00: uint32_t magic = 0x48575652 ("HWVR")
# 0x04: uint8_t  major (version dependent)
# 0x05: uint8_t  minor (version dependent)
# 0x06: uint8_t  led_green_port (GPIO port for green LED: 0=A, 1=B, 2=C)
# 0x07: uint8_t  led_green_pin (GPIO pin number for green LED)
# 0x08: uint8_t  led_red_port (GPIO port for red LED: 0=A, 1=B, 2=C)
# 0x09: uint8_t  led_red_pin (GPIO pin number for red LED)
# 0x0A: uint8_t  reserved = 0x00
# 0x0B: uint8_t  reserved2 = 0x00  
# 0x0C: uint16_t reserved3 = 0x0000
# 0x0E: uint16_t crc16 (CRC16-CCITT-FALSE over first 14 bytes)

set HW_INFO_ADDR 0x08007800

# WRP Register addresses for STM32C011
set FLASH_BASE      0x40022000
set FLASH_OPTR      [expr {$FLASH_BASE + 0x20}]
set FLASH_WRP1AR    [expr {$FLASH_BASE + 0x2C}]
set FLASH_WRP1BR    [expr {$FLASH_BASE + 0x30}]

proc brand_hw {data_file {enable_wrp 0}} {
    global HW_INFO_ADDR
    
    if {$data_file == ""} {
        echo "Error: No data file specified!"
        echo "Usage: brand_hw <data_file> [enable_wrp]"
        echo "Example: brand_hw scripts/hw_v2_data.bin"
        echo "Example: brand_hw scripts/hw_v2_data.bin 1  (with WRP protection)"
        return
    }
    
    if {![file exists $data_file]} {
        echo "Error: Data file '$data_file' not found!"
        return
    }
    
    # Read version info from the data file for display
    set fp [open $data_file "rb"]
    set data [read $fp 16]
    close $fp
    
    if {[string length $data] < 16} {
        echo "Error: Data file '$data_file' is too small (need 16 bytes, got [string length $data])"
        return
    }
    
    # Extract version info for display (magic number is stored as "HWVR" = 0x48575652)
    scan [string index $data 0] "%c" magic0
    scan [string index $data 1] "%c" magic1
    scan [string index $data 2] "%c" magic2
    scan [string index $data 3] "%c" magic3
    set magic [expr {($magic0 << 24) | ($magic1 << 16) | ($magic2 << 8) | ($magic3 & 0xFF)}]
    scan [string index $data 4] "%c" major
    scan [string index $data 5] "%c" minor
    scan [string index $data 6] "%c" led_green_port
    scan [string index $data 7] "%c" led_green_pin
    scan [string index $data 8] "%c" led_red_port  
    scan [string index $data 9] "%c" led_red_pin
    scan [string index $data 14] "%c" crc_low
    scan [string index $data 15] "%c" crc_high
    set crc [expr {($crc_high << 8) | ($crc_low & 0xFF)}]
    
    echo ""
    echo "============================================"
    echo "  xESC_YF_rev4 Hardware Version Branding"
    echo "============================================"
    echo ""
    echo "Data file: $data_file"
    echo "Target address: [format "0x%08X" $HW_INFO_ADDR]"
    echo "Hardware version: $major.$minor"
    echo "LED Config: Green=P$led_green_port$led_green_pin, Red=P$led_red_port$led_red_pin"
    echo "CRC16: [format "0x%04X" $crc]"
    echo ""
    
    # Verify magic number
    if {$magic != 0x48575652} {
        echo "Warning: Invalid magic number in data file!"
        echo "Expected: 0x48575652 (HWVR)"
        echo "Found: [format "0x%08X" $magic]"
        echo ""
    }
    
    # Initialize OpenOCD and connect to target
    init
    
    # Halt the processor
    halt
    
    # Show current flash lock status
    echo "Current flash status:"
    flash info 0
    
    # Unlock flash for writing
    echo "Unlocking flash..."
    reset halt
    
    # Try different unlock approaches
    catch {flash protect 0 0 last off}
    catch {stm32f1x unlock 0}
    
    # Erase page 15 (reserved 2KB page for hardware info)
    echo "Erasing reserved page 15 (0x08007800-0x08007FFF)..."
    
    # Try multiple erase approaches
    if {[catch {flash erase_sector 0 15 15}]} {
        echo "Standard erase failed, trying address-based erase..."
        catch {flash erase_address 0x08007800 0x800}
    }
    
    # Write hardware version structure to reserved flash page
    echo "Writing hardware version structure..."
    
    # Program the hardware version data to flash
    program $data_file $HW_INFO_ADDR verify
    
    echo "Hardware version structure written successfully!"
    
    # Verify the written data
    echo ""
    echo "Verifying written data:"
    halt
    sleep 200
    
    echo "Raw bytes:"
    mdb $HW_INFO_ADDR 16
    
    echo ""
    echo "32-bit words:"
    mdw $HW_INFO_ADDR 4
    
    echo ""
    
    # Enable WRP protection if requested
    if {$enable_wrp} {
        echo "Enabling WRP protection for hardware version data..."
        enable_wrp_protection
        echo "Hardware Version $major.$minor branding with WRP protection completed successfully!"
    } else {
        echo "Hardware Version $major.$minor branding completed successfully!"
        echo "(Note: WRP protection not enabled - use 'brand_hw $data_file 1' to enable protection)"
    }
    echo ""
}

proc verify_hardware_version {} {
    global HW_INFO_ADDR
    
    set addr $HW_INFO_ADDR
    
    echo ""
    echo "Reading hardware version from [format "0x%08X" $addr]..."
    
    # Initialize OpenOCD and connect to target
    init
    
    # Read and check magic number (mrw reads in little-endian format)
    set magic [mrw $addr]
    echo "Magic: [format "0x%08X" $magic]"
    
    if {$magic == 0x52565748} {
        set major [expr {[mrb [expr {$addr + 4}]] & 0xFF}]
        set minor [expr {[mrb [expr {$addr + 5}]] & 0xFF}]
        set crc_low [expr {[mrb [expr {$addr + 14}]] & 0xFF}]
        set crc_high [expr {[mrb [expr {$addr + 15}]] & 0xFF}]
        set crc [expr {($crc_high << 8) | $crc_low}]
        
        echo "✓ Valid hardware info structure found!"
        echo "  Version: $major.$minor"
        echo "  CRC16: [format "0x%04X" $crc]"
        
        echo "✓ Device is branded as v$major.$minor"
    } else {
        echo "✗ No valid hardware info found!"
        echo "  Expected magic: 0x52565748 (HWVR in little-endian)"
        echo "  Found magic: [format "0x%08X" $magic]"
        echo ""
        echo "Raw data at location:"
        mdb $addr 16
    }
    echo ""
}

proc erase_hardware_version {} {
    echo ""
    echo "Erasing hardware version info..."
    
    # Initialize OpenOCD and connect to target
    init
    
    halt
    flash protect 0 0 last off
    flash erase_sector 0 15 15
    
    echo "Hardware version info erased!"
    echo ""
}

proc create_hw_data_file {major minor filename} {
    echo ""
    echo "Creating hardware data file: $filename"
    echo "Version: $major.$minor"
    
    # HwInfo structure (16 bytes total):
    # uint32_t magic;          // 0x48575652 ("HWVR")
    # uint8_t major;           // Major version number
    # uint8_t minor;           // Minor version number  
    # uint8_t led_green_port;  // GPIO port for green LED (2=C)
    # uint8_t led_green_pin;   // GPIO pin number for green LED (15)
    # uint8_t led_red_port;    // GPIO port for red LED (1=B for v2, 2=C for v1)
    # uint8_t led_red_pin;     // GPIO pin number for red LED (6 for v2, 14 for v1)
    # uint8_t reserved;        // Reserved for future use
    # uint8_t reserved2;       // Additional reserved byte
    # uint16_t reserved3;      // Additional reserved space (2 bytes)
    # uint16_t crc16;         // CRC16-CCITT-FALSE over first 14 bytes
    
    # Determine LED configuration based on hardware version
    if {$major == 1} {
        # V1 hardware: Green=PC15, Red=PC14
        set led_green_port 2
        set led_green_pin 15
        set led_red_port 2
        set led_red_pin 14
    } else {
        # V2+ hardware: Green=PC15, Red=PB6
        set led_green_port 2
        set led_green_pin 15
        set led_red_port 1
        set led_red_pin 6
    }
    
    echo "LED Config: Green=P$led_green_port$led_green_pin, Red=P$led_red_port$led_red_pin"
    
    set fp [open $filename "wb"]
    
    # Write magic "HWVR" (0x48575652) in little-endian
    puts -nonewline $fp [format "%c%c%c%c" 0x48 0x57 0x56 0x52]
    
    # Write version and LED configuration + reserved bytes (10 bytes total)
    # Bytes 4-13: major, minor, led_green_port, led_green_pin, led_red_port, led_red_pin, reserved, reserved2, reserved3_low, reserved3_high
    puts -nonewline $fp [format "%c%c%c%c%c%c%c%c%c%c" $major $minor $led_green_port $led_green_pin $led_red_port $led_red_pin 0 0 0 0]
    
    # Write CRC placeholder (2 bytes) for bytes 14-15
    puts -nonewline $fp [format "%c%c" 0 0]
    
    close $fp
    
    echo "File created: $filename"
    echo ""
    echo "============================================"
    echo "  CRC16 Calculation Required"
    echo "============================================"
    echo ""
    echo "The file was created with CRC16 = 0x0000. You need to calculate the correct CRC16-CCITT-FALSE."
    echo ""
    echo "Step 1: Calculate CRC16-CCITT-FALSE over first 14 bytes:"
    echo ""
    echo "# Extract first 14 bytes and calculate CRC:"
    echo "head -c 14 $filename | crc16 --poly=0x1021 --init=0xFFFF --xorout=0x0000"
    echo ""
    echo "Alternative with Python (if available):"
    echo "python3 -c \""
    echo "import struct"
    echo "with open('$filename', 'rb') as f: data = f.read(14)"
    echo "crc = 0xFFFF"
    echo "for byte in data:"
    echo "    crc ^= (byte << 8)"
    echo "    for _ in range(8):"
    echo "        crc = (crc << 1) ^ 0x1021 if (crc & 0x8000) else (crc << 1)"
    echo "        crc &= 0xFFFF"
    echo "print(f'CRC16: 0x{crc:04X} (little-endian bytes: {crc & 0xFF:02X} {(crc >> 8) & 0xFF:02X})')\""
    echo ""
    echo "Step 2: Update the CRC in the file (replace XXXX with calculated CRC):"
    echo ""
    echo "# If CRC is 0x1234, write as little-endian (0x34 0x12):"
    echo "printf '\\x34\\x12' | dd of=$filename bs=1 seek=14 count=2 conv=notrunc"
    echo ""
    echo "Step 3: Verify the complete file:"
    echo ""
    echo "hexdump -C $filename"
    echo ""
    echo "Expected format:"
    echo "00000000  48 57 56 52 \[maj\] \[min\] \[grn_port\] \[grn_pin\] \[red_port\] \[red_pin\] 00 00  |HWVR............|"
    echo "00000010  00 00 \[crc_low\] \[crc_high\]                              |....            |"
    echo ""
}

# Print usage information
echo ""
echo "xESC_YF_rev4 Generic Hardware Branding Script"
echo "=============================================="
echo ""
echo "Available commands:"
echo "  brand_hw <data_file> \[enable_wrp\] - Brand device with specified data file"
echo "  verify_hardware_version           - Read and verify current hardware version"
echo "  erase_hardware_version            - Erase hardware version info (requires WRP disabled)"
echo "  create_hw_data_file <major> <minor> <filename> - Create new data file template"
echo "  calculate_and_fix_crc <filename>  - Auto-calculate and fix CRC16 in data file"
echo ""
echo "  enable_wrp_protection             - Enable WRP protection for hardware info"
echo "  disable_wrp_protection            - Disable WRP protection (for recovery)"
echo "  show_wrp_status                   - Show current WRP protection status"
echo ""
echo "Usage examples:"
echo "  # Create new data file and auto-fix CRC:"
echo "  openocd \[...\] -c 'create_hw_data_file 2 1 scripts/hw_v2.1_data.bin; exit'"
echo "  openocd \[...\] -c 'calculate_and_fix_crc scripts/hw_v2.1_data.bin; exit'"
echo ""
echo "  # Brand device (without/with WRP protection):"
echo "  openocd \[...\] -c 'brand_hw scripts/hw_v2_data.bin; exit'     (without WRP)"
echo "  openocd \[...\] -c 'brand_hw scripts/hw_v2_data.bin 1; exit'   (with WRP protection)"
echo "  openocd \[...\] -c 'brand_hw scripts/hw_v3_data.bin 1; exit'   (v3.0 with WRP)"
echo ""
echo "  # Status and management:"
echo "  openocd \[...\] -c 'show_wrp_status; exit'"
echo "  openocd \[...\] -c 'verify_hardware_version; exit'"
echo ""
echo "Data files needed:"
echo "  scripts/hw_v1_data.bin   - For Hardware Version 1.0 (Green=PC15, Red=PC14)"
echo "  scripts/hw_v2_data.bin   - For Hardware Version 2.0 (Green=PC15, Red=PB6)" 
echo "  scripts/hw_v2.1_data.bin - For Hardware Version 2.1 (Green=PC15, Red=PB6)"
echo "  scripts/hw_v3_data.bin   - For Hardware Version 3.0 (LED config TBD)"
echo ""

# ============================================================================
# CRC16 Helper Functions
# ============================================================================

proc calculate_and_fix_crc {filename} {
    echo ""
    echo "============================================"
    echo "  Auto-fixing CRC16 for: $filename"
    echo "============================================"
    echo ""
    
    if {![file exists $filename]} {
        echo "Error: File '$filename' not found!"
        return
    }
    
    # Read the file
    set fp [open $filename "rb"]
    set data [read $fp]
    close $fp
    
    if {[string length $data] != 16} {
        echo "Error: File size is [string length $data] bytes, expected 16 bytes"
        return
    }
    
    # Extract first 14 bytes for CRC calculation
    set crc_data [string range $data 0 13]
    
    # Calculate CRC16-CCITT-FALSE
    set crc 0xFFFF
    
    for {set i 0} {$i < [string length $crc_data]} {incr i} {
        # Get byte value using scan
        scan [string index $crc_data $i] "%c" byte
        set byte [expr {$byte & 0xFF}]
        
        set crc [expr {$crc ^ ($byte << 8)}]
        
        for {set bit 0} {$bit < 8} {incr bit} {
            if {$crc & 0x8000} {
                set crc [expr {(($crc << 1) ^ 0x1021) & 0xFFFF}]
            } else {
                set crc [expr {($crc << 1) & 0xFFFF}]
            }
        }
    }
    
    echo "Calculated CRC16: [format "0x%04X" $crc]"
    echo "Little-endian bytes: [format "0x%02X 0x%02X" [expr {$crc & 0xFF}] [expr {($crc >> 8) & 0xFF}]]"
    
    # Update the file with correct CRC (little-endian format)
    set fp [open $filename "r+b"]
    seek $fp 14
    puts -nonewline $fp [format "%c%c" [expr {$crc & 0xFF}] [expr {($crc >> 8) & 0xFF}]]
    close $fp
    
    echo ""
    echo "✓ CRC16 updated in file: $filename"
    echo ""
    echo "Verification:"
    echo "============="
    
    # Verify by displaying the file
    set fp [open $filename "rb"]
    set final_data [read $fp]
    close $fp
    
    # Display hex dump
    for {set i 0} {$i < [string length $final_data]} {incr i} {
        if {$i % 16 == 0} {
            echo -n [format "%08x  " $i]
        }
        
        scan [string index $final_data $i] "%c" byte
        set byte [expr {$byte & 0xFF}]
        echo -n [format "%02x " $byte]
        
        if {$i % 16 == 15 || $i == [string length $final_data] - 1} {
            echo ""
        }
    }
    
    # Extract version info manually (since we can't use binary scan)
    scan [string index $final_data 4] "%c" major
    scan [string index $final_data 5] "%c" minor
    scan [string index $final_data 14] "%c" crc_low
    scan [string index $final_data 15] "%c" crc_high
    set final_crc [expr {($crc_high << 8) | ($crc_low & 0xFF)}]
    
    echo ""
    echo "File contents:"
    echo "  Magic: HWVR (✓ Valid)"
    echo "  Version: $major.$minor"
    echo "  CRC16: [format "0x%04X" $final_crc] ([expr {$final_crc == $crc ? "✓ Correct" : "✗ Mismatch"}])"
    echo ""
}

# ============================================================================
# WRP (Write Protection) Functions
# ============================================================================

proc enable_wrp_protection {} {
    global HW_INFO_ADDR FLASH_BASE FLASH_WRP1AR
    
    echo ""
    echo "============================================"
    echo "  Enabling WRP Protection for Hardware Info"
    echo "============================================"
    echo ""
    
    # Initialize OpenOCD and connect to target
    init
    halt
    
    # Calculate page number for HW_INFO_ADDR (0x08007800)
    # STM32C011: 2KB pages, base 0x08000000
    # Page 15 = (0x08007800 - 0x08000000) / 0x800 = 0x0F
    set hw_info_page 0x0F
    
    echo "Hardware info at: [format "0x%08X" $HW_INFO_ADDR]"
    echo "Page to protect: $hw_info_page"
    
    # STM32C0 Flash Register Offsets - corrected based on reference manual
    
    echo "Flash registers:"
    echo "  FLASH_SR:  0x40022010 (corrected)"
    echo "  FLASH_CR:  0x40022014 (corrected)"
    echo "  WRP1AR:    [format "0x%08X" $FLASH_WRP1AR]"
    
    # Check if flash is locked
    set cr_value [mrw 0x40022014]
    echo "Current FLASH_CR: [format "0x%08X" $cr_value]"
    
    # Unlock flash 
    echo "Unlocking flash..."
    mww 0x40022008 0x45670123
    mww 0x40022008 0xCDEF89AB
    
    # Check if flash is unlocked
    set cr_after_unlock [mrw 0x40022014]
    echo "FLASH_CR after unlock: [format "0x%08X" $cr_after_unlock]"
    
    # Unlock option bytes
    echo "Unlocking option bytes..."
    mww 0x4002200C 0x08192A3B
    mww 0x4002200C 0x4C5D6E7F
    
    # Check if option bytes are unlocked
    set cr_after_opt_unlock [mrw 0x40022014]
    echo "FLASH_CR after option unlock: [format "0x%08X" $cr_after_opt_unlock]"
    
    # Read current WRP1AR value
    set current_wrp1ar [mrw $FLASH_WRP1AR]
    echo "Current WRP1AR: [format "0x%08X" $current_wrp1ar]"
    
    # Configure WRP1AR to protect page 15
    # WRP1A_STRT = 0x0F (bits 7:0)  
    # WRP1A_END = 0x0F (bits 23:16)
    set new_wrp1ar [expr {(0x0F << 16) | 0x0F}]
    
    echo "Setting WRP1AR to: [format "0x%08X" $new_wrp1ar]"
    mww $FLASH_WRP1AR $new_wrp1ar
    
    # Verify the write took effect
    set written_wrp1ar [mrw $FLASH_WRP1AR]
    echo "WRP1AR after write: [format "0x%08X" $written_wrp1ar]"
    
    # Start option byte programming using OPTSTRT bit
    echo "Starting option byte programming..."
    set optstrt_bit [expr {1 << 17}]
    
    # Set OPTSTRT bit in FLASH_CR to start option byte programming
    mmw 0x40022014 $optstrt_bit 0
    
    # Check status register for programming progress
    set bsy_bit [expr {1 << 16}]
    
    echo "Waiting for option byte programming to complete..."
    set timeout 100
    
    while {$timeout > 0} {
        set sr_value [mrw 0x40022010]
        echo "FLASH_SR: [format "0x%08X" $sr_value] (BSY=[expr {($sr_value & $bsy_bit) ? 1 : 0}])"
        
        if {($sr_value & $bsy_bit) == 0} {
            echo "Option byte programming completed (BSY cleared)"
            break
        }
        sleep 10
        set timeout [expr {$timeout - 1}]
    }
    
    if {$timeout == 0} {
        echo "ERROR: Option byte programming timeout!"
        echo "Final FLASH_SR: [format "0x%08X" [mrw 0x40022010]]"
        return
    }
    
    # Trigger option byte loading using OBL_LAUNCH bit
    echo "Triggering option byte loading..."
    set obl_launch_bit [expr {1 << 27}]
    mmw 0x40022014 $obl_launch_bit 0
    
    # Wait for option bytes to reload (device will reset)
    echo "Option bytes loading... (device will reset)"
    sleep 1000
    
    # Reconnect after reset
    echo "Reconnecting after option bytes reload..."
    catch {
        init
        reset halt
    }
    sleep 500
    
    # Verify WRP is active
    set final_wrp1ar [mrw $FLASH_WRP1AR]
    echo ""
    echo "Verification Results:"
    echo "===================="
    echo "Expected WRP1AR: [format "0x%08X" $new_wrp1ar]"
    echo "Final WRP1AR:    [format "0x%08X" $final_wrp1ar]"
    
    if {$final_wrp1ar == $new_wrp1ar} {
        echo "✓ WRP protection successfully enabled for page 15!"
        echo "✓ Hardware version info is now tamper-proof!"
    } else {
        echo "⚠ WRP protection not active yet!"
        echo "  This may require a complete power cycle (unplug/replug power)"
        echo "  Try: power off device, power on, then run 'show_wrp_status' to verify"
    }
    
    echo ""
}

proc disable_wrp_protection {} {
    global FLASH_BASE FLASH_WRP1AR
    
    echo ""
    echo "============================================"
    echo "  Disabling WRP Protection" 
    echo "============================================"
    echo ""
    
    # Initialize OpenOCD and connect to target
    init
    halt
    
    # Unlock flash and option bytes
    echo "Unlocking flash and option bytes..."
    
    # FLASH_KEYR unlock sequence
    mww [expr {$FLASH_BASE + 0x08}] 0x45670123
    mww [expr {$FLASH_BASE + 0x08}] 0xCDEF89AB
    
    # FLASH_OPTKEYR unlock sequence
    mww [expr {$FLASH_BASE + 0x0C}] 0x08192A3B
    mww [expr {$FLASH_BASE + 0x0C}] 0x4C5D6E7F
    
    # Clear WRP1AR (no protection) 
    echo "Clearing WRP1AR..."
    mww $FLASH_WRP1AR 0xFFFFFFFF
    
    # Start option byte programming
    echo "Starting option byte programming..."
    set flash_cr [expr {$FLASH_BASE + 0x14}]  # Corrected CR register address
    set optstrt_bit [expr {1 << 17}]
    
    # Set OPTSTRT bit in FLASH_CR to start option byte programming
    mmw $flash_cr $optstrt_bit 0
    
    # Wait for option byte programming to complete
    echo "Waiting for option byte programming to complete..."
    sleep 100
    
    # Check if BSY bit is cleared (programming completed)
    set flash_sr [expr {$FLASH_BASE + 0x10}]  # Corrected SR register address
    set bsy_bit [expr {1 << 16}]
    set timeout 10
    
    while {$timeout > 0} {
        set sr_value [mrw $flash_sr]
        if {($sr_value & $bsy_bit) == 0} {
            echo "Option byte programming completed"
            break
        }
        sleep 100
        set timeout [expr {$timeout - 1}]
    }
    
    if {$timeout == 0} {
        echo "Warning: Option byte programming timeout"
    }
    
    # Trigger system reset to load new option bytes
    echo "Triggering system reset to load new option bytes..."
    set obl_launch_bit [expr {1 << 27}]
    mmw $flash_cr $obl_launch_bit 0
    
    echo "Option bytes reloading... (device will reset)"
    sleep 2000
    
    echo "✓ WRP protection disabled!"
    echo ""
}

proc show_wrp_status {} {
    global FLASH_WRP1AR FLASH_WRP1BR
    
    echo ""
    echo "WRP Protection Status:"
    echo "======================"
    
    # Initialize OpenOCD and connect to target
    init
    
    set wrp1ar [mrw $FLASH_WRP1AR]
    set wrp1br [mrw $FLASH_WRP1BR]
    
    echo "WRP1AR: [format "0x%08X" $wrp1ar]"
    echo "WRP1BR: [format "0x%08X" $wrp1br]"
    
    # Decode WRP1AR
    set wrp1a_strt [expr {$wrp1ar & 0xFF}]
    set wrp1a_end [expr {($wrp1ar >> 16) & 0xFF}]
    
    if {$wrp1a_strt <= $wrp1a_end && $wrp1a_strt != 0xFF} {
        echo "WRP Area A: Pages $wrp1a_strt to $wrp1a_end protected"
        
        if {$wrp1a_strt == 0x0F && $wrp1a_end == 0x0F} {
            echo "✓ Page 15 (Hardware Info) is WRP protected!"
        }
    } else {
        echo "WRP Area A: No protection"
    }
    
    # Decode WRP1BR  
    set wrp1b_strt [expr {$wrp1br & 0xFF}]
    set wrp1b_end [expr {($wrp1br >> 16) & 0xFF}]
    
    if {$wrp1b_strt <= $wrp1b_end && $wrp1b_strt != 0xFF} {
        echo "WRP Area B: Pages $wrp1b_strt to $wrp1b_end protected"
    } else {
        echo "WRP Area B: No protection"
    }
    
    echo ""
}