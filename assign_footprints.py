#!/usr/bin/env python3
"""
Assign footprints to PomeloCore schematic components
Minimum 0805 for SMD resistors and small capacitors
"""

import re
import sys

# Footprint mappings based on component type and value
FOOTPRINT_MAP = {
    # Resistors - all 0805 minimum
    'R': 'Resistor_SMD:R_0805_2012Metric',

    # Capacitors - based on value
    # Small values (pF, nF, <1uF) -> 0805
    # 1uF-10uF -> 0805 or 1206 depending on voltage
    # >10uF -> 1206 or larger
    'C_small': 'Capacitor_SMD:C_0805_2012Metric',      # <1uF
    'C_medium': 'Capacitor_SMD:C_0805_2012Metric',     # 1uF-4.7uF
    'C_large': 'Capacitor_SMD:C_1206_3216Metric',      # 10uF+

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
    'U15': 'Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical',  # 803-87-006-10-001101

    # Diodes
    'D_SOD123': 'Diode_SMD:D_SOD-123',                 # 1N4148W
    'D_SOT23': 'Package_TO_SOT_SMD:SOT-23',            # BAS21HT1G, BAT54C
    'D_TVS': 'Package_SO:SO-8_3.9x4.9mm_P1.27mm',      # NUP2114UCMR6T1G
    'D_SMA': 'Diode_SMD:D_SMA',                        # SS34

    # Transistors
    'Q_SOT23': 'Package_TO_SOT_SMD:SOT-23',            # BC847, MMBT5551, AO3401A

    # LEDs
    'LED1': 'LED_SMD:LED_0805_2012Metric',

    # Inductors
    'L_0805': 'Inductor_SMD:L_0805_2012Metric',
    'L_1210': 'Inductor_SMD:L_1210_3225Metric',

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
}

def get_capacitor_footprint(value_str):
    """Determine capacitor footprint based on value"""
    value_str = value_str.lower()

    # Parse value
    if 'pf' in value_str or 'nf' in value_str:
        return FOOTPRINT_MAP['C_small']
    elif 'uf' in value_str or 'µf' in value_str:
        # Extract number
        match = re.search(r'([\d.]+)', value_str)
        if match:
            val = float(match.group(1))
            if val >= 10:
                return FOOTPRINT_MAP['C_large']
            else:
                return FOOTPRINT_MAP['C_medium']

    return FOOTPRINT_MAP['C_small']  # Default

def get_diode_footprint(value_str):
    """Determine diode footprint based on part number"""
    value_str = value_str.lower()

    if 'nup2114' in value_str:
        return FOOTPRINT_MAP['D_TVS']
    elif 'bas21' in value_str or 'bat54' in value_str:
        return FOOTPRINT_MAP['D_SOT23']
    elif 'ss34' in value_str:
        return FOOTPRINT_MAP['D_SMA']
    else:  # 1N4148W and others
        return FOOTPRINT_MAP['D_SOD123']

def get_inductor_footprint(value_str):
    """Determine inductor footprint based on value"""
    if '10u' in value_str.lower():
        return FOOTPRINT_MAP['L_1210']  # Larger for power inductor
    else:
        return FOOTPRINT_MAP['L_0805']

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
        return FOOTPRINT_MAP.get(ref, 'Package_TO_SOT_SMD:SOT-23-5')  # Default to SOT-23-5

    # Diodes
    if ref.startswith('D'):
        return get_diode_footprint(value)

    # Transistors
    if ref.startswith('Q'):
        return FOOTPRINT_MAP['Q_SOT23']

    # LEDs
    if ref.startswith('LED'):
        return FOOTPRINT_MAP['LED1']

    # Inductors
    if ref.startswith('L'):
        return get_inductor_footprint(value)

    # Test Points
    if ref.startswith('TP'):
        return FOOTPRINT_MAP['TP']

    # Direct lookup for connectors, crystals, buttons
    return FOOTPRINT_MAP.get(ref, None)

def process_schematic(input_file, output_file):
    """Process schematic file and assign footprints"""

    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()

    # Pattern to find symbol instances with Reference and Value
    # We need to find the property "Footprint" and update it

    # Find all symbol instances
    symbol_pattern = r'(\(symbol\s+\(lib_id[^)]+\).*?(?=\n\t\(symbol|\n\t\(wire|\n\t\(junction|\Z))'

    updates = 0

    def update_footprint(match):
        nonlocal updates
        symbol_text = match.group(0)

        # Extract reference
        ref_match = re.search(r'\(property "Reference" "([^"]+)"', symbol_text)
        if not ref_match:
            return symbol_text

        ref = ref_match.group(1)

        # Skip power symbols
        if ref.startswith('#PWR'):
            return symbol_text

        # Extract value
        value_match = re.search(r'\(property "Value" "([^"]+)"', symbol_text)
        value = value_match.group(1) if value_match else ""

        # Get appropriate footprint
        footprint = assign_footprint(ref, value)

        if footprint:
            # Replace empty footprint with assigned one
            new_text = re.sub(
                r'(\(property "Footprint" )"([^"]*)"',
                f'\\1"{footprint}"',
                symbol_text
            )
            if new_text != symbol_text:
                updates += 1
                print(f"Assigned {ref} ({value}): {footprint}")
            return new_text

        return symbol_text

    # Process the content
    new_content = re.sub(symbol_pattern, update_footprint, content, flags=re.DOTALL)

    # Write output
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(new_content)

    print(f"\nTotal footprints assigned: {updates}")
    return updates

if __name__ == '__main__':
    input_file = "/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/Polemo Core 2.kicad_sch"
    output_file = input_file  # Overwrite original

    print("Assigning footprints to PomeloCore schematic...")
    print("Minimum size for SMD passives: 0805")
    print("")

    updates = process_schematic(input_file, output_file)

    print(f"\nDone! Updated {updates} components.")
