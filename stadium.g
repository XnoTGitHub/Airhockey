Include:'airhockey_table.g'


Prefix: "L_"
Include: '../scenarios/panda_moveGripper.g'

Prefix: "R_"
Include: '../scenarios/panda_moveGripper.g'

camera_red(ground){
    Q:<t(-0.01 .8 1.1) d(160 1 0 0)d(180 1 0 0)d(180 0 0 1)>,
    #Q:<t(-0.01 .2 1.8) d(0 90 -90 90)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}
camera_green(ground){
    Q:<t(0.01 -.8 1.1) d(20 1 0 0)>,
    #Q:<t(-0.01 .2 1.8) d(0 90 -90 90)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}

Edit R_panda_link0 (ground) { Q:<t(0 1.3 0) d(0 0 90 1)> }
Edit L_panda_link0 (ground) { Q:<t(0 -1.3 0) d(0 90 -30 1)> }

Edit L_finger1{ joint:rigid }
Edit L_finger2{ joint:rigid }
Edit R_finger1{ joint:rigid }
Edit R_finger2{ joint:rigid }

Include:'pusher.g'
