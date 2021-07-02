
stick_red (world){
    shape:cylinder,X:<t(0. 0.8 1)>, size:[.2, 0.04], color:[.3 .3 .3]
    #mass:10
    contact, logical:{ }
    friction:.1
    joint:rigid
}
#surf (stick)	{  shape:cylinder, size:[0.02, 0.06],, mass:0.2 Q:<t(0 0. -0.05)>}

stick_green (world){
    shape:cylinder,X:<t(0. -0.8 1)>, size:[.2, 0.04], color:[.3 .3 .3]
    #mass:10
    contact, logical:{ }
    friction:.1
    joint:rigid
}
