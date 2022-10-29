def select_mission(WP_index, switch_WP):

    # for start change gear D
    if switch_WP==1 and WP_index<=1:
        mission=11

    # for left lamp
    elif switch_WP==1 and WP_index>=2 and WP_index<=3:
        mission=1

    elif switch_WP==1 and WP_index==11:
        mission=44

    # climb, stop 3s and go
    elif switch_WP==1 and WP_index==12:
        mission=4

    # Traffic signal mission
    elif (switch_WP==1 and WP_index>=36 and WP_index<=40) or (switch_WP==2 and WP_index>=17 and WP_index<=22) or (switch_WP==3 and WP_index>=1 and WP_index<=2): # stop line, distance
        mission=5

    # No GPS
    elif switch_WP==2 and WP_index>=4 and WP_index<=13:
        mission=6

    # before recognition pedestrian dcc
    elif (switch_WP==1 and WP_index==24) or (switch_WP==2 and WP_index==23) or (switch_WP==3 and WP_index==19):
        mission=77 # pre

    # recognition pedestrian
    elif (switch_WP==1 and WP_index>=25 and WP_index<=28) or (switch_WP==2 and WP_index>=24 and WP_index<=27) or (switch_WP==3 and WP_index>=20 and WP_index<=21):
        mission=7

    # velocity acc
    elif switch_WP==3 and WP_index>=28 and WP_index<=34:
        mission=8

    # right lamp on before stop line
    elif switch_WP==3 and WP_index>=52 and WP_index<=54:
        mission=9
    
    # gear Parking
    elif switch_WP==3 and WP_index>=55:
        mission=10


    else:
        mission=0

    return mission