$Action = 0;//[0:Full Case, 1:Base, 2:Top]

module RoundedBlock($XDim = 43, $YDim = 43, $ZDim = 9.6, $CurveD = 4)
{
  hull()
  {
    translate([-($XDim - $CurveD)/ 2, -($YDim  - $CurveD)/ 2, -($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
    translate([($XDim - $CurveD)/ 2, -($YDim  - $CurveD)/ 2, -($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
    translate([-($XDim - $CurveD)/ 2, ($YDim  - $CurveD)/ 2, -($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
    translate([($XDim - $CurveD)/ 2, ($YDim  - $CurveD)/ 2, -($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
    translate([-($XDim - $CurveD)/ 2, -($YDim  - $CurveD)/ 2, ($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
    translate([($XDim - $CurveD)/ 2, -($YDim  - $CurveD)/ 2, ($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
    translate([-($XDim - $CurveD)/ 2, ($YDim  - $CurveD)/ 2, ($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
    translate([($XDim - $CurveD)/ 2, ($YDim  - $CurveD)/ 2, ($ZDim - $CurveD) / 2])
      sphere( d = $CurveD, $fn = 50);
  }
}

module USBConnector($Clearance = .5)
{
  $Rounding = 1.5;
  
  translate([0, 19.4 / 2, 0])
  {
    //Shroud
    RoundedBlock($XDim = 16 + $Clearance, $YDim = 19.4 + $Clearance, $ZDim = 9 + $Clearance, $CurveD = $Rounding);
    //Strain relief body
    translate([0, (25 - 19.4) / 2, 0])
      RoundedBlock($XDim = 10.5 + $Clearance, $YDim = 25 + $Clearance, $ZDim = 9 + $Clearance, $CurveD = $Rounding);
    //Strain relief cable
    translate([0, -((19.4 + $Clearance) / 2) + 1, 0])
      rotate(-90, [1, 0, 0])
        cylinder(d = 8.2 + $Clearance, h = 37.2 + $Clearance - 1);
    //Cable
    translate([0, -(19.4 + $Clearance) / 2, 0])
      rotate(-90, [1, 0, 0])
        cylinder(d = 5 + $Clearance, h = 50 + $Clearance);
    //Plug
    translate([0, -(12.2 + 19.4) / 2, 0])
      cube([12 + $Clearance, 12.2 + $Clearance, 4.5 + $Clearance], center = true);
  }
}

module FTCDriverHubModedCase()
{
  import("Driver_Hub_Case.stl", convexity=3);
}

module FTCDriverHubModedBottom()
{
  $OriginalDepth = 6;
  $Depth = 16.5;
  $Rounding = 3;
  
  //Add text to the bottom of the case
  difference()
  {
    import("Driver_Hub_Bottom.stl", convexity=3);
    translate([38.0, -7, -7])
      linear_extrude(6)
      {
        mirror([1, 0])
//        text("MERCS", size = 14);
        text("", size = 14);
      }
  }
  
  //Cable tail holder
  difference()
  {
    translate([4.5, -65, ($Depth / 2) - $OriginalDepth])
      RoundedBlock($XDim = 90, $YDim = 50, $ZDim = $Depth, $CurveD = $Rounding);
    
    translate([0, -43 - 3, 8.8])
    {
      //Type-C power cable opening
      translate([36-43.5, -22, 0])
        RoundedBlock($XDim = 13, $YDim = 50, $ZDim = 7, $CurveD = $Rounding);
      //Joystick openings
      translate([36, 0, 0])
        rotate(180, [1, 0, 0])
          USBConnector($Clearance = .5);
      translate([-26.8, 0, 0])
        rotate(180, [1, 0, 0])
          USBConnector($Clearance = .5);
    }
    //Main case clearance
    translate([0, - (20 / 2) -26.55, 20 / 2])
      cube([150, 20, 20], center = true);
  }
}

module FTCDriverHubModed()
{
  FTCDriverHubModedBottom();
  FTCDriverHubModedCase();
}

if      ($Action == 0) {FTCDriverHubModed();}
else if ($Action == 1) {FTCDriverHubModedBottom();}
else if ($Action == 2) {FTCDriverHubModedCase();}

