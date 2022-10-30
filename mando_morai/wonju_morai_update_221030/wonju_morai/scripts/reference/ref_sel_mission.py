# mission 1 : At stop line wait 5s -> go(only once)
# mission 2 : Rotary
# mission 3 : Traffic light signal recognize
# mission 4 : Static object --> lane change
# mission 5 : Dynamic objecy --> emergency stop -> go


def select_mission(WP_index,switch_WP): # specific range WP --> change road type

    if switch_WP==1 and WP_index ==15: # mission 1 init == 13
        current_mission=1
    elif (switch_WP==1 and WP_index>=34 and WP_index<=55) or (switch_WP==2 and WP_index>=33 and WP_index<=54): # init 29, 48 / 30 51 
        current_mission=2
    elif (switch_WP==1 and WP_index==78) or (switch_WP==2 and WP_index==15): #init 72 14
        current_mission=3
    elif (switch_WP==2 and WP_index>=95 and WP_index<=114) or (switch_WP==3 and WP_index>=2 and WP_index<=25): # init 86 97 / 5 ??
        current_mission=4
    elif (switch_WP==2 and WP_index>=163 and WP_index<=183) or (switch_WP==3 and WP_index>=77 and WP_index<=92): # init 151 164 / 75 85
        current_mission=5
    elif (switch_WP==1 and WP_index>=32 and WP_index<=33) or (switch_WP==2 and WP_index>=31 and WP_index<=32):
        current_mission=6 # mission 2-2


    else:
        current_mission=0

    return current_mission