#!/usr/bin/env python3
"""
Assign footprints to PomeloCore schematic components - Version 2
Properly handles KiCad 9 format with symbol instances
Minimum 0805 for SMD resistors and small capacitors
"""

import re
import sys

# Footprint mappings
FOOTPRINT_MAP = {
    # Resistors - all 0805 minimum
    'R': 'Resistor_SMD:R_0805_2012Metric',

    # ICs - specific packages
    'U1': 'Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.15x5.15mm',  # ATSAML21G18B
    'U2': 'Package_TO_SOT_SMD:SOT-23-5',               # XC6209F332MR-G LDO
    'U3': 'Package_SO:SOIC-16_3.9x9.9mm_P1.27mm',      # MC14043BDR2G
    'U4': 'Package_SO:MSOP-8_3x3mm_P0.65mm',           # LMV793MFX/NOPB
    'U5': 'Package_TO_SOT_SMD:SOT-23-5',               # TPS7A2030PDBVR
    'U6': 'Package_TO_SOT_SMD:SOT-23-5',               # TLV3201AIDBVT
    'U7': 'Package_TO_SOT_SMD:SOT-23-5',               # OPA357AIDBVR
    'U8': 'Package_TO_SOT_SMD:SOT-23-6',               # TLR341G-GTR
    'U9': 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',       # MF-ASML020/6-2
    'U10': 'Package_TO_SOT_SMD:SOT-353_SC-70-5',       # 74LVC1G08GV
    'U11': 'Package_TO_SOT_SMD:SOT-23-5',              # LT3014BHVES5
    'U12': 'Package_SO:VSSOP-8_2.3x2mm_P0.5mm',        # TXU0202DCUR
    'U13': 'Package_TO_SOT_SMD:SOT-23-5',              # TLV7031DBVR
    'U14': 'Package_TO_SOT_SMD:SOT-23-6',              # TLR341G-GTR
    'U15': 'Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical',

    # Connectors
    'USBC1': 'Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12',
    'J-LINK1': 'Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical',
    'P1': 'Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical',
    'P2': 'Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical',
    'H2': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'H3': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'H5': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'MEAS1': 'Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical',

    # Test Points
    'TP': 'TestPoint:TestPoint_Pad_D1.5mm',

    # Crystal
    'X1': 'Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm',

    # Button
    'BUT1': 'Button_Switch_SMD:SW_SPST_TL3342',

    # LED
    'LED1': 'LED_SMD:LED_0805_2012Metric',
}

def get_capacitor_footprint(value_str):
    """Determine capacitor footprint based on value"""
    value_str = value_str.lower()

    # Parse value
    if 'pf' in value_str or 'nf' in value_str:
        return 'Capacitor_SMD:C_0805_2012Metric'
    elif 'uf' in value_str or 'µf' in value_str:
        match = re.search(r'([\d.]+)', value_str)
        if match:
            val = float(match.group(1))
            if val >= 10:
                return 'Capacitor_SMD:C_1206_3216Metric'
            else:
                return 'Capacitor_SMD:C_0805_2012Metric'

    return 'Capacitor_SMD:C_0805_2012Metric'

def get_diode_footprint(value_str):
    """Determine diode footprint based on part number"""
    value_str = value_str.lower()

    if 'nup2114' in value_str:
        return 'Package_SO:SO-8_3.9x4.9mm_P1.27mm'
    elif 'bas21' in value_str or 'bat54' in value_str:
        return 'Package_TO_SOT_SMD:SOT-23'
    elif 'ss34' in value_str:
        return 'Diode_SMD:D_SMA'
    else:
        return 'Diode_SMD:D_SOD-123'

def get_inductor_footprint(value_str):
    """Determine inductor footprint based on value"""
    if '10u' in value_str.lower() or '470' in value_str:
        return 'Inductor_SMD:L_1210_3225Metric'
    else:
        return 'Inductor_SMD:L_0805_2012Metric'

def get_transistor_footprint(value_str):
    """All transistors are SOT-23"""
    return 'Package_TO_SOT_SMD:SOT-23'

def assign_footprint(ref, value):
    """Assign appropriate footprint based on reference designator and value"""

    # Power symbols don't need footprints
    if ref.startswith('#PWR'):
        return None

    # Resistors
    if ref.startswith('R'):
        return FOOTPRINT_MAP['R']

    # Capacitors
    if ref.startswith('C'):
        return get_capacitor_footprint(value)

    # ICs
    if ref.startswith('U'):
        return FOOTPRINT_MAP.get(ref, 'Package_TO_SOT_SMD:SOT-23-5')

    # Diodes
    if ref.startswith('D'):
        return get_diode_footprint(value)

    # Transistors
    if ref.startswith('Q'):
        return get_transistor_footprint(value)

    # LEDs
    if ref.startswith('LED'):
        return FOOTPRINT_MAP['LED1']

    # Inductors
    if ref.startswith('L'):
        return get_inductor_footprint(value)

    # Test Points
    if ref.startswith('TP'):
        return FOOTPRINT_MAP['TP']

    # Direct lookup
    return FOOTPRINT_MAP.get(ref, None)

def process_schematic(input_file, output_file):
    """Process schematic file and assign footprints to symbol instances"""

    with open(input_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()

    updates = 0
    i = 0

    while i < len(lines):
        line = lines[i]

        # Look for symbol blocks
        if line.strip().startswith('(symbol'):
            # Find this symbol's reference and value
            ref = None
            value = None
            footprint_line_idx = None

            # Scan ahead in this symbol block
            j = i + 1
            block_depth = 1

            while j < len(lines) and block_depth > 0:
                curr_line = lines[j]

                # Track nesting depth
                block_depth += curr_line.count('(') - curr_line.count(')')

                # Extract reference
                if '(reference "' in curr_line:
                    ref_match = re.search(r'\(reference "([^"]+)"', curr_line)
                    if ref_match:
                        ref = ref_match.group(1)

                # Extract value
                if '(property "Value" "' in curr_line and value is None:
                    val_match = re.search(r'\(property "Value" "([^"]+)"', curr_line)
                    if val_match:
                        value = val_match.group(1)

                # Find footprint property line
                if '(property "Footprint"' in curr_line and footprint_line_idx is None:
                    footprint_line_idx = j

                j += 1

                # If we found all we need and passed the footprint line, process it
                if ref and footprint_line_idx and block_depth == 0:
                    break

            # Assign footprint if we found everything
            if ref and footprint_line_idx is not None:
                footprint = assign_footprint(ref, value or "")

                if footprint:
                    # Update the footprint line
                    old_line = lines[footprint_line_idx]
                    new_line = re.sub(
                        r'(\(property "Footprint" )"([^"]*)"',
                        f'\\1"{footprint}"',
                        old_line
                    )

                    if new_line != old_line:
                        lines[footprint_line_idx] = new_line
                        updates += 1
                        print(f"Assigned {ref} ({value}): {footprint}")

        i += 1

    # Write output
    with open(output_file, 'w', encoding='utf-8') as f:
        f.writelines(lines)

    print(f"\nTotal footprints assigned: {updates}")
    return updates

if __name__ == '__main__':
    input_file = "/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/Polemo Core 2.kicad_sch"
    output_file = input_file

    print("Assigning footprints to PomeloCore schematic (v2)...")
    print("Minimum size for SMD passives: 0805\n")

    updates = process_schematic(input_file, output_file)

    print(f"\nDone! Updated {updates} components.")
