#!/usr/bin/env python3
"""
Assign footprints to PomeloCore schematic - FINAL VERSION
Assigns to actual component instances in the schematic (not library definitions)
Minimum 0805 for SMD resistors and small capacitors
"""

import re

FOOTPRINT_MAP = {
    'R': 'Resistor_SMD:R_0805_2012Metric',
    'U1': 'Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.15x5.15mm',
    'U2': 'Package_TO_SOT_SMD:SOT-23-5',
    'U3': 'Package_SO:SOIC-16_3.9x9.9mm_P1.27mm',
    'U4': 'Package_SO:MSOP-8_3x3mm_P0.65mm',
    'U5': 'Package_TO_SOT_SMD:SOT-23-5',
    'U6': 'Package_TO_SOT_SMD:SOT-23-5',
    'U7': 'Package_TO_SOT_SMD:SOT-23-5',
    'U8': 'Package_TO_SOT_SMD:SOT-23-6',
    'U9': 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
    'U10': 'Package_TO_SOT_SMD:SOT-353_SC-70-5',
    'U11': 'Package_TO_SOT_SMD:SOT-23-5',
    'U12': 'Package_SO:VSSOP-8_2.3x2mm_P0.5mm',
    'U13': 'Package_TO_SOT_SMD:SOT-23-5',
    'U14': 'Package_TO_SOT_SMD:SOT-23-6',
    'U15': 'Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical',
    'USBC1': 'Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12',
    'J-LINK1': 'Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical',
    'P1': 'Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical',
    'P2': 'Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical',
    'H2': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'H3': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'H5': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'MEAS1': 'Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical',
    'TP': 'TestPoint:TestPoint_Pad_D1.5mm',
    'X1': 'Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm',
    'BUT1': 'Button_Switch_SMD:SW_SPST_TL3342',
    'LED1': 'LED_SMD:LED_0805_2012Metric',
}

def get_capacitor_footprint(value_str):
    value_str = value_str.lower()
    if 'pf' in value_str or 'nf' in value_str:
        return 'Capacitor_SMD:C_0805_2012Metric'
    elif 'uf' in value_str or 'µf' in value_str:
        match = re.search(r'([\d.]+)', value_str)
        if match and float(match.group(1)) >= 10:
            return 'Capacitor_SMD:C_1206_3216Metric'
    return 'Capacitor_SMD:C_0805_2012Metric'

def get_diode_footprint(value_str):
    value_str = value_str.lower()
    if 'nup2114' in value_str:
        return 'Package_SO:SO-8_3.9x4.9mm_P1.27mm'
    elif 'bas21' in value_str or 'bat54' in value_str:
        return 'Package_TO_SOT_SMD:SOT-23'
    elif 'ss34' in value_str:
        return 'Diode_SMD:D_SMA'
    return 'Diode_SMD:D_SOD-123'

def get_inductor_footprint(value_str):
    if '10u' in value_str.lower() or '470' in value_str:
        return 'Inductor_SMD:L_1210_3225Metric'
    return 'Inductor_SMD:L_0805_2012Metric'

def assign_footprint(ref, value):
    if ref.startswith('#PWR'):
        return None
    if ref.startswith('R'):
        return FOOTPRINT_MAP['R']
    if ref.startswith('C'):
        return get_capacitor_footprint(value)
    if ref.startswith('U'):
        return FOOTPRINT_MAP.get(ref, 'Package_TO_SOT_SMD:SOT-23-5')
    if ref.startswith('D'):
        return get_diode_footprint(value)
    if ref.startswith('Q'):
        return 'Package_TO_SOT_SMD:SOT-23'
    if ref.startswith('LED'):
        return FOOTPRINT_MAP['LED1']
    if ref.startswith('L'):
        return get_inductor_footprint(value)
    if ref.startswith('TP'):
        return FOOTPRINT_MAP['TP']
    return FOOTPRINT_MAP.get(ref, None)

def process_schematic(input_file, output_file):
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Split into lines for processing
    lines = content.split('\n')

    updates = 0
    i = 0
    in_symbol_instance = False
    current_ref = None
    current_value = None
    footprint_line_idx = None

    new_lines = []

    while i < len(lines):
        line = lines[i]

        # Detect start of symbol instance (not lib_symbol definition)
        if line.strip().startswith('(symbol') and '(lib_id' in line:
            in_symbol_instance = True
            current_ref = None
            current_value = None
            footprint_line_idx = None

        if in_symbol_instance:
            # Look for Reference property
            if '\t\t(property "Reference"' in line:
                # Next line should have the reference value
                ref_match = re.search(r'"([^"]+)"', lines[i])
                if ref_match:
                    current_ref = ref_match.group(1)

            # Look for Value property
            if '\t\t(property "Value"' in line and current_value is None:
                val_match = re.search(r'"([^"]+)"', lines[i])
                if val_match:
                    current_value = val_match.group(1)

            # Look for Footprint property line
            if '\t\t(property "Footprint"' in line:
                footprint_line_idx = i

                # Assign footprint if we have all the info
                if current_ref:
                    footprint = assign_footprint(current_ref, current_value or "")

                    if footprint:
                        # Replace the footprint on this line
                        new_line = re.sub(
                            r'(\(property "Footprint" )"([^"]*)"',
                            f'\\1"{footprint}"',
                            line
                        )

                        if new_line != line:
                            line = new_line
                            updates += 1
                            print(f"Assigned {current_ref} ({current_value}): {footprint}")

            # Check for end of symbol instance
            if line.strip() == ')' and current_ref:
                in_symbol_instance = False

        new_lines.append(line)
        i += 1

    # Write output
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(new_lines))

    print(f"\nTotal footprints assigned: {updates}")
    return updates

if __name__ == '__main__':
    input_file = "/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/Polemo Core 2.kicad_sch"
    output_file = input_file

    print("Assigning footprints to PomeloCore component instances...")
    print("Minimum size for SMD passives: 0805\n")

    updates = process_schematic(input_file, output_file)

    print(f"\nDone! Updated {updates} components.")
