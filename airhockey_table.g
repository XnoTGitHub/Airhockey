world {}


ground (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[5. 5. .1 .02], color:[.3 .3 .3]
    contact, logical:{ }
    #friction:.1 
}

leg_one (ground){
	shape:ssBox, Q:<t(0 .8 0.15)>, size:[.8 .2 .3 .1], color:[.2 .2 .2]
    contact, logical:{ }
    #friction:.1
}
leg_two (ground){
	shape:ssBox, Q:<t(0 -.8 0.15)>, size:[.8 .2 .3 .1], color:[.2 .2 .2]
    contact, logical:{ }
    #friction:.1
}
plate (leg_two){
	shape:ssBox, Q:<t(0 .8 0.15)>, size:[1.2 2.2 .05 0], color:[.2 .2 .2]
    contact, logical:{ }
    #friction:0.01
}
band_long_one (plate){
	shape:ssBox, Q:<t(0.575 0 0.025)>, size:[0.05 2.2 .075 .02], color:[.2 .2 .2]
    contact, logical:{ }
    #friction:.1
    restitution: 1.
}
band_long_two (plate){
	shape:ssBox, Q:<t(-0.575 0 0.025)>, size:[0.05 2.2 .075 .02], color:[.2 .2 .2]
    contact, logical:{ }
    #friction:.1
    restitution: 1.
}
band_short_one (plate){
	shape:ssBox, Q:<t(0 1.075 0.025)>, size:[1.2 0.05 .075 .02], color:[.2 .2 .2]
    contact, logical:{ }
    #friction:.1
    restitution: 1.
}
band_short_two (plate){
	shape:ssBox, Q:<t(0 -1.075 0.025)>, size:[1.2 0.05 .075 .02], color:[.2 .2 .2]
    contact, logical:{ }
    #friction:.1
    restitution: 1.

