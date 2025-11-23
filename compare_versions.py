#!/usr/bin/env python3
"""
Compare Pomelo Core V1.2 (from PDF analysis) with V1.3 (KiCad schematic)
"""

import re

def extract_v13_components(sch_file):
    """Extract component values and references from V1.3 schematic"""

    with open(sch_file, 'r', encoding='utf-8') as f:
        content = f.read()

    components = {}

    # Find all symbol instances with their values
    pattern = r'\(property "Reference" "([^"]+)".*?\(property "Value" "([^"]+)"'

    matches = re.finditer(pattern, content, re.DOTALL)

    for match in matches:
        ref = match.group(1)
        value = match.group(2)

        if not ref.startswith('#PWR'):
            components[ref] = value

    return components

# Manual extraction from V1.2 PDF
V12_COMPONENTS = {
    # Resistors
    'R1': '27k',
    'R2': '5.1k',
    'R3': '47k',
    'R4': '1k',
    'R5': '220k',
    'R6': '1M',
    'R7': '5.1k',
    'R8': '330k',
    'R9': '1k',
    'R10': '100',
    'R11': '330k',
    'R12': '330k',
    'R13': '100k',
    'R14': '120M',
    'R15': '5.6M',
    'R16': '100k',
    'R17': '1k',
    'R18': '100',
    'R19': '22k',
    'R20': '100',
    'R21': '6.8k',
    'R22': '30k',
    'R23': '10k',
    'R25': '4.7M',
    'R26': '2.2k',
    'R27': '6.8k',
    'R28': '100',
    'R29': '1k',
    'R30': '1k',
    'R31': '510',
    'R32': '100',
    'R33': '1k',
    'R34': '200k',
    'R35': '47k',
    'R36': '47k',
    'R37': '10k',
    'R38': '0',
    'R39': '27k',
    'R40': '1k',

    # Capacitors
    'C1': '1uF',
    'C2': '10nF',
    'C3': '4.7uF',
    'C4': '10uF',
    'C5': '10uF',
    'C6': '10uF',
    'C7': '100nF',
    'C8': '10nF',
    'C9': '30pF',
    'C10': '4.7nF',
    'C11': '100nF',
    'C12': '100nF',
    'C13': '1uF',
    'C14': '1uF',
    'C15': '1nF',
    'C16': '1nF',
    'C17': '10nF',
    'C18': '100nF',
    'C19': '4.7uF',
    'C20': '4.7uF',
    'C21': '100nF',
    'C22': '10nF',
    'C23': '100nF',
    'C24': '1nF',
    'C25': '1uF',
    'C26': '100nF',
    'C27': '100nF',
    'C28': '1uF',
    'C29': '1nF',
    'C30': '1nF',
    'C31': '1uF',
    'C32': '1uF',
    'C33': '1uF',
    'C34': '1nF',
    'C35': '100nF',
    'C36': '100nF',
    'C37': '1uF',
    'C38': '4.7uF',
    'C39': '4.7uF',
    'C40': '100nF',
    'C41': '100nF',
    'C42': '18pF',
    'C43': '18pF',
    'C44': '1uF',
    'C45': '4.7uF',
    'C46': '100nF',
    'C47': '4.7uF',
    'C48': '4.7uF',
    'C49': '4.7uF',
    'C50': '100nF',
    'C51': '100nF',
    'C52': '4.7uF',
    'C53': '100nF',
    'C54': '4.7uF',
    'C55': '100nF',
    'C56': '100nF',
    'C57': '10uF',

    # Inductors
    'L1': 'SDR0503-103JL',
    'L2': '10uH',
    'L3': '470',

    # ICs
    'U1': 'ATSAML21G18B-AUT',
    'U2': 'XC6209F332MR-G',
    'U3': 'MC14043BDR2G',
    'U4': 'LMV793MFX/NOPB',
    'U5': 'TPS7A2030PDBVR',
    'U6': 'TLV3201AIDBVT',
    'U7': 'OPA357AIDBVR',
    'U8': 'TLR341G-GTR',
    'U10': '74LVC1G08GV,125',
    'U11': 'LT3014BHVES5#TRMPBF',
    'U12': 'TXU0202DCUR',
    'U13': 'TLV7031DBVR',
    'U14': 'TLR341G-GTR',

    # Diodes
    'D1': 'NUP2114UCMR6T1G',
    'D2': 'BAS21HT1G',
    'D3': '1N4148W_C81598',
    'D4': '1N4148W_C81598',
    'D5': '1N4148W_C81598',
    'D7': 'BAT54C,215',
    'D9': 'SS34_C8678',

    # Transistors
    'Q1': 'BC847_C2910145',
    'Q2': 'MMBT5551LT1G',
    'Q3': 'AO3401A',

    # Crystal
    'X1': '32.768kHz',

    # LED
    'LED1': '19-217/GHC-YR1S2/3T',

    # Headers (simplified)
    'H1': 'KH-2.54PH180-1X3P-L11.5',
    'H2': 'KH-2.54PH180-1X6P-L11.5',
    'H3': 'KH-2.54PH180-1X6P-L11.5',
    'H5': 'KH-2.54PH180-1X7P-L11.5',
    'H7': 'KH-2.54PH180-1X3P-L11.5',
}

def compare_versions(v13_file):
    """Compare V1.2 and V1.3"""

    print("="*80)
    print("POMELO CORE VERSION COMPARISON: V1.2 → V1.3")
    print("="*80)
    print()

    # Get V1.3 components
    v13_comps = extract_v13_components(v13_file)

    # Find differences
    changed = []
    added = []
    removed = []

    # Check what changed
    all_refs = set(list(V12_COMPONENTS.keys()) + list(v13_comps.keys()))

    for ref in sorted(all_refs):
        v12_val = V12_COMPONENTS.get(ref, None)
        v13_val = v13_comps.get(ref, None)

        if v12_val and v13_val:
            if v12_val != v13_val:
                changed.append((ref, v12_val, v13_val))
        elif v13_val and not v12_val:
            added.append((ref, v13_val))
        elif v12_val and not v13_val:
            removed.append((ref, v12_val))

    # Print results
    if changed:
        print("📝 CHANGED COMPONENTS:")
        print("-" * 80)
        for ref, old, new in changed:
            print(f"  {ref:10s}  {old:25s} → {new:25s}")
        print()

    if added:
        print("➕ ADDED COMPONENTS (new in V1.3):")
        print("-" * 80)
        for ref, val in added:
            print(f"  {ref:10s}  {val}")
        print()

    if removed:
        print("➖ REMOVED COMPONENTS (present in V1.2, not in V1.3):")
        print("-" * 80)
        for ref, val in removed:
            print(f"  {ref:10s}  {val}")
        print()

    if not changed and not added and not removed:
        print("✓ No differences found - versions are identical")
        print()

    # Summary
    print("="*80)
    print("SUMMARY:")
    print(f"  Changed:  {len(changed)} components")
    print(f"  Added:    {len(added)} components")
    print(f"  Removed:  {len(removed)} components")
    print("="*80)

if __name__ == '__main__':
    v13_file = "/home/rob/Documents/Safecast/PomeloCore-KiCad/Polemo Core/Polemo Core 2/Polemo Core 2.kicad_sch"

    compare_versions(v13_file)
