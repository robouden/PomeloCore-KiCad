#!/usr/bin/env python3
import re

file_path = "/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/Polemo Core 2.kicad_sch"

FP = {
    'R': 'Resistor_SMD:R_0805_2012Metric',
    'C': 'Capacitor_SMD:C_0805_2012Metric',
    'C_LRG': 'Capacitor_SMD:C_1206_3216Metric',
    'U1': 'Package_DFN_QFN:QFN-48-1EP_7x7mm_P0.5mm_EP5.15x5.15mm',
    'U2': 'Package_TO_SOT_SMD:SOT-23-5', 'U3': 'Package_SO:SOIC-16_3.9x9.9mm_P1.27mm',
    'U4': 'Package_SO:MSOP-8_3x3mm_P0.65mm', 'U5': 'Package_TO_SOT_SMD:SOT-23-5',
    'U6': 'Package_TO_SOT_SMD:SOT-23-5', 'U7': 'Package_TO_SOT_SMD:SOT-23-5',
    'U8': 'Package_TO_SOT_SMD:SOT-23-6', 'U9': 'Package_SO:SOIC-8_3.9x4.9mm_P1.27mm',
    'U10': 'Package_TO_SOT_SMD:SOT-353_SC-70-5', 'U11': 'Package_TO_SOT_SMD:SOT-23-5',
    'U12': 'Package_SO:VSSOP-8_2.3x2mm_P0.5mm', 'U13': 'Package_TO_SOT_SMD:SOT-23-5',
    'U14': 'Package_TO_SOT_SMD:SOT-23-6', 'U15': 'Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Vertical',
    'USBC1': 'Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12',
    'J-LINK1': 'Connector_PinHeader_2.54mm:PinHeader_2x05_P2.54mm_Vertical',
    'P1': 'Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical',
    'P2': 'Connector_PinHeader_2.54mm:PinHeader_2x02_P2.54mm_Vertical',
    'H2': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'H3': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'H5': 'Connector_PinHeader_2.54mm:PinHeader_1x07_P2.54mm_Vertical',
    'MEAS1': 'Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical',
    'TP': 'TestPoint:TestPoint_Pad_D1.5mm', 'X1': 'Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm',
    'BUT1': 'Button_Switch_SMD:SW_SPST_TL3342', 'LED': 'LED_SMD:LED_0805_2012Metric',
    'D': 'Diode_SMD:D_SOD-123', 'D_TVS': 'Package_SO:SO-8_3.9x4.9mm_P1.27mm',
    'D_SOT': 'Package_TO_SOT_SMD:SOT-23', 'D_SMA': 'Diode_SMD:D_SMA',
    'Q': 'Package_TO_SOT_SMD:SOT-23', 'L': 'Inductor_SMD:L_0805_2012Metric',
    'L_LRG': 'Inductor_SMD:L_1210_3225Metric',
}

print("Reading file...")
with open(file_path, 'r') as f:
    lines = f.readlines()

print(f"File has {len(lines)} lines")

in_inst = False
ref = val = None
updates = 0

for i in range(len(lines)):
    line = lines[i]

    # Skip library definitions
    if i < 30000:
        continue

    # Detect symbol instance
    if '\t(symbol' in line and '(lib_id' in line:
        in_inst = True
        ref = val = None

    if in_inst:
        # Extract reference
        if '(property "Reference"' in line:
            m = re.search(r'"Reference" "([^"]+)"', line)
            if m: ref = m.group(1)

        # Extract value
        if '(property "Value"' in line and not val:
            m = re.search(r'"Value" "([^"]+)"', line)
            if m: val = m.group(1)

        # Update footprint
        if '(property "Footprint" ""' in line and ref:
            fp = None
            if ref.startswith('#PWR'):
                pass
            elif ref.startswith('R'):
                fp = FP['R']
            elif ref.startswith('C'):
                if val and ('10u' in val.lower() or '10µ' in val.lower()):
                    fp = FP['C_LRG']
                else:
                    fp = FP['C']
            elif ref.startswith('U'):
                fp = FP.get(ref, 'Package_TO_SOT_SMD:SOT-23-5')
            elif ref.startswith('D'):
                if val:
                    if 'nup2114' in val.lower():
                        fp = FP['D_TVS']
                    elif 'bas21' in val.lower() or 'bat54' in val.lower():
                        fp = FP['D_SOT']
                    elif 'ss34' in val.lower():
                        fp = FP['D_SMA']
                    else:
                        fp = FP['D']
                else:
                    fp = FP['D']
            elif ref.startswith('Q'):
                fp = FP['Q']
            elif ref.startswith('L'):
                if val and '10u' in val.lower():
                    fp = FP['L_LRG']
                else:
                    fp = FP['L']
            elif ref.startswith('TP'):
                fp = FP['TP']
            elif ref.startswith('LED'):
                fp = FP['LED']
            else:
                fp = FP.get(ref)

            if fp:
                lines[i] = line.replace('(property "Footprint" ""', f'(property "Footprint" "{fp}"')
                updates += 1
                print(f"{ref} ({val}): {fp}")

        # End of symbol
        if line.strip() == ')' and in_inst and ref:
            in_inst = False

print(f"\nWriting file...")
with open(file_path, 'w') as f:
    f.writelines(lines)

print(f"Done! Assigned {updates} footprints")
