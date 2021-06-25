set key autotitle columnheader
set title 'costReport ( plotting sqrt(costs) )'
plot 'z.costReport' \
      u (($0+1)/20):1 w l lw 3 lc 1 lt 1 \
  ,'' u (($0+1)/20):2 w l lw 3 lc 2 lt 1 \
  ,'' u (($0+1)/20):3 w l lw 3 lc 3 lt 1 \
  ,'' u (($0+1)/20):4 w l lw 3 lc 4 lt 1 \
  ,'' u (($0+1)/20):5 w l lw 3 lc 5 lt 1 \
  ,'' u (($0+1)/20):6 w l lw 3 lc 6 lt 1 \
  ,'' u (($0+1)/20):7 w l \
  ,'' u (($0+1)/20):8 w l \
  ,'' u (($0+1)/20):9 w l \
  ,'' u (($0+1)/20):10 w l \
  ,'' u (($0+1)/20):11 w l \
  ,'' u (($0+1)/20):12 w l \

