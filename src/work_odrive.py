import odrive
import odrive.enums

odrives = odrive.find_any(find_multiple = 6)

def check_errors(odrives):
    bad_odrives = []
    for od in odrives:
        print(f'Checking {od.serial_number}')
        has_error = False
        j1_error = od.axis0.error
        j2_error = od.axis1.error
        print(j1_error,j2_error)
        has_error |= j1_error > 0 
        has_error |= j2_error > 0
        if has_error:
            bad_odrives.append(od) 
    return bad_odrives

evil = check_errors(odrives)
if len(evil) >0: 
    for od in evil:
        print(f'bad odrive found: {od.serial_number}')
        od.reboot()
else:
    print('Evil has been vanquished')