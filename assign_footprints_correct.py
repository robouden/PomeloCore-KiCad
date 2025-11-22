#!/usr/bin/env python3
"""
Assign footprints to PomeloCore schematic - CORRECT VERSION
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

def get_cap_fp(v):
    v = v.lower()
    if 'pf' in v or 'nf' in v: return 'Capacitor_SMD:C_0805_2012Metric'
    if 'uf' in v or 'µf' in v:
        m = re.search(r'([\d.]+)', v)
        if m and float(m.group(1)) >= 10: return 'Capacitor_SMD:C_1206_3216Metric'
    return 'Capacitor_SMD:C_0805_2012Metric'

def get_diode_fp(v):
    v = v.lower()
    if 'nup2114' in v: return 'Package_SO:SO-8_3.9x4.9mm_P1.27mm'
    if 'bas21' in v or 'bat54' in v: return 'Package_TO_SOT_SMD:SOT-23'
    if 'ss34' in v: return 'Diode_SMD:D_SMA'
    return 'Diode_SMD:D_SOD-123'

def get_ind_fp(v):
    return 'Inductor_SMD:L_1210_3225Metric' if ('10u' in v.lower() or '470' in v) else 'Inductor_SMD:L_0805_2012Metric'

def assign_fp(ref, val):
    if ref.startswith('#PWR'): return None
    if ref.startswith('R'): return FOOTPRINT_MAP['R']
    if ref.startswith('C'): return get_cap_fp(val)
    if ref.startswith('U'): return FOOTPRINT_MAP.get(ref, 'Package_TO_SOT_SMD:SOT-23-5')
    if ref.startswith('D'): return get_diode_fp(val)
    if ref.startswith('Q'): return 'Package_TO_SOT_SMD:SOT-23'
    if ref.startswith('LED'): return FOOTPRINT_MAP['LED1']
    if ref.startswith('L'): return get_ind_fp(val)
    if ref.startswith('TP'): return FOOTPRINT_MAP['TP']
    return FOOTPRINT_MAP.get(ref, None)

with open("/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/Polemo Core 2.kicad_sch", 'r') as f:
    lines = f.readlines()

updates = 0
i = 0
in_inst = False
ref = val = None

while i < len(lines):
    line = lines[i]
    
    if '\t(symbol' in line and '(lib_id' in line:
        in_inst = True
        ref = val = None
    
    if in_inst:
        if '\t\t(property "Reference" "' in line:
            m = re.search(r'"Reference" "([^"]+)"', line)
            if m: ref = m.group(1)
        
        if '\t\t(property "Value" "' in line and not val:
            m = re.search(r'"Value" "([^"]+)"', line)
            if m: val = m.group(1)
        
        if '\t\t(property "Footprint"' in line and ref:
            fp = assign_fp(ref, val or "")
            if fp:
                new_line = re.sub(r'(\(property "Footprint" )"([^"]*)"', f'\\1"{fp}"', line)
                if new_line != line:
                    lines[i] = new_line
                    updates += 1
                    print(f"{ref} ({val}): {fp}")
        
        if line.strip() == ')' and ref:
            in_inst = False
    
    i += 1

with open("/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/Polemo Core 2.kicad_sch", 'w') as f:
    f.writelines(lines)

print(f"\nTotal: {updates} footprints assigned")
