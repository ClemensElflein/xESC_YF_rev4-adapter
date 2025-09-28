# Hardware Version 2.0 Branding Script for xESC_YF_rev4
# This script writes hardware version info to flash memory for runtime detection
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
# 0x04: uint8_t  major = 2
# 0x05: uint8_t  minor = 0  
# 0x06: uint16_t reserved = 0x0000
# 0x08: uint64_t reserved2 = 0x0000000000000000
# 0x0E: uint16_t crc16 (CRC16-CCITT-FALSE over first 14 bytes)
#
# Binary Data File Creation:
# The hw_v2_data.bin file contains the hardware version structure and can be created with:
# echo -ne '\x52\x56\x57\x48\x02\x00\x00\x00\x00\x00\x00\x00\x00\x00\x0D\x8B' > scripts/hw_v2_data.bin
#
# Binary breakdown:
# 52 56 57 48 = "HWVR" magic (little-endian)
# 02 00       = Version 2.0 (major=2, minor=0)
# 00 00       = Reserved (2 bytes)
# 00 00 00 00 00 00 00 00 = Reserved2 (8 bytes)
# 0D 8B       = CRC16-CCITT-FALSE checksum (little-endian)

set HW_INFO_ADDR 0x08007800

proc brand_hw_v2 {} {
    global HW_INFO_ADDR
    
    echo ""
    echo "============================================"
    echo "  xESC_YF_rev4 Hardware Version 2.0 Branding"
    echo "============================================"
    echo ""
    
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
    echo "Writing hardware version structure at [format "0x%08X" $HW_INFO_ADDR]..."
    
    set addr $HW_INFO_ADDR
    
    # The cleanest approach: use a pre-created binary file
    # Hardware version structure (16 bytes): Magic + Version + Reserved + CRC
    # Magic: 52 56 57 48 (HWVR), Version: 02 00, Reserved: 00 00 00 00 00 00 00 00, CRC: 0D 8B
    
    set hw_data_file "scripts/hw_v2_data.bin"
    
    if {![file exists $hw_data_file]} {
        echo "Error: Hardware data file $hw_data_file not found!"
        echo "Please create it first using: echo -ne '\\x52\\x56\\x57\\x48\\x02\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x00\\x0D\\x8B' > $hw_data_file"
        return
    }
    
    # Program the hardware version data to flash
    program $hw_data_file $addr verify
    
    echo "Hardware version structure written successfully!"
    
    # Verify the written data
    echo ""
    echo "Verifying written data:"
    echo "Raw bytes:"
    mdb $addr 16
    
    echo ""
    echo "32-bit words:"
    mdw $addr 4
    
    echo ""
    echo "Hardware Version 2.0 branding completed successfully!"
    echo "Device is now branded as HW_V2"
    echo ""
}

proc verify_hardware_version {} {
    global HW_INFO_ADDR
    
    set addr $HW_INFO_ADDR
    
    echo ""
    echo "Reading hardware version from [format "0x%08X" $addr]..."
    
    # Initialize OpenOCD and connect to target
    init
    
    # Read and check magic number
    set magic [mrw $addr]
    echo "Magic: [format "0x%08X" $magic]"
    
    if {$magic == 0x48575652} {
        set major [mrb [expr {$addr + 4}]]
        set minor [mrb [expr {$addr + 5}]]
        set crc_low [mrb [expr {$addr + 14}]]
        set crc_high [mrb [expr {$addr + 15}]]
        set crc [expr {($crc_high << 8) | $crc_low}]
        
        echo "✓ Valid hardware info structure found!"
        echo "  Version: $major.$minor"
        echo "  CRC16: [format "0x%04X" $crc]"
        
        if {$major == 2 && $minor == 0} {
            echo "✓ Device is correctly branded as HW_V2"
        } else {
            echo "⚠ Unexpected version: $major.$minor (expected: 2.0)"
        }
    } else {
        echo "✗ No valid hardware info found!"
        echo "  Expected magic: 0x48575652 (HWVR)"
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
    echo "Device will now default to HW_V1 behavior"
    echo ""
}

# Print usage information
echo ""
echo "xESC_YF_rev4 Hardware Branding Script Loaded"
echo "============================================="
echo ""
echo "Available commands:"
echo "  brand_hw_v2             - Brand device as Hardware Version 2.0"
echo "  verify_hardware_version - Read and verify current hardware version"
echo "  erase_hardware_version  - Erase hardware version (revert to HW_V1 default)"
echo ""
echo "Usage example:"
echo "  openocd -f interface/stlink.cfg -f target/stm32c0x.cfg -f scripts/brand_hw_v2.tcl -c 'brand_hw_v2; exit'"
echo ""