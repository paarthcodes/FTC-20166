$fn = 50;

$ReturnOffset = 0;
$SpacerHeight = 3;
$ShowBearingOutlines = true;
$BearingOutlineD = 12.5;

$PlateThickness = 0.125 * 25.4;
$BearingHeight = 10.2;//4.5 for V-grove cord


module RoundedBlock($XDim = 10, $YDim = 10, $ZDim = 10, $D1 = 0, $D2 = 0, $D3 = 0, $D4 = 0, $D5 = 0, $D6 = 0, $D7 = 0, $D8 = 0)
{
  $fn = 20;
  hull()
  {
    translate([($XDim - $D1) / 2, ($YDim - $D1) / 2, ($ZDim - $D1)/ 2])
      sphere(d = $D1);
    translate([-($XDim - $D2) / 2, ($YDim - $D2) / 2, ($ZDim - $D2)/ 2])
      sphere(d = $D2);
    translate([-($XDim - $D3) / 2, -($YDim - $D3) / 2, ($ZDim - $D3)/ 2])
      sphere(d = $D3);
    translate([($XDim - $D4) / 2, -($YDim - $D4) / 2, ($ZDim - $D4)/ 2])
      sphere(d = $D4);
    
    translate([($XDim - $D5) / 2, ($YDim - $D5) / 2, -($ZDim - $D5)/ 2])
      sphere(d = $D5);
    translate([-($XDim - $D6) / 2, ($YDim - $D6) / 2, -($ZDim - $D6)/ 2])
      sphere(d = $D6);
    translate([-($XDim - $D7) / 2, -($YDim - $D7) / 2, -($ZDim - $D7)/ 2])
      sphere(d = $D7);
    translate([($XDim - $D8) / 2, -($YDim - $D8) / 2, -($ZDim - $D8)/ 2])
      sphere(d = $D8);
    
  }
}

module RoundedPlate($XDim = 10, $YDim = 10, $ZDim = 10, $D1 = 0, $D2 = 0, $D3 = 0, $D4 = 0)
{
  $fn = 20;
  hull()
  {
    translate([($XDim - $D1) / 2, ($YDim - $D1) / 2, -$ZDim / 2])
      cylinder(d = $D1, h = $ZDim);
    translate([-($XDim - $D2) / 2, ($YDim - $D2) / 2, -$ZDim / 2])
      cylinder(d = $D2, h = $ZDim);
    translate([-($XDim - $D3) / 2, -($YDim - $D3) / 2, -$ZDim / 2])
      cylinder(d = $D3, h = $ZDim);
    translate([($XDim - $D4) / 2, -($YDim - $D4) / 2, -$ZDim / 2])
      cylinder(d = $D4, h = $ZDim);
  }
}

module MisumiLiftSlideDrillTemplate2($Stages = 1)
{
  $fn = 20;
  $Clearance = 0.2;
  $YOffset = 5 + ($Clearance / 2);
  $XOffset = 12;//
  $XHoleSpacing = 33;
  $YSpacing = 16;
  
  difference()
  {
    RoundedBlock($XDim = 53, $YDim = ($Stages * $YSpacing) + 6, $ZDim = 27, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    translate([-2, 0, 0])
    {
      cube([53, ($Stages * $YSpacing) + $Clearance, 20.2], center = true);
      for (i = [0:$Stages - 1])
      {
        translate([(53 / 2) - $XOffset, (($Stages * $YSpacing) / 2) - $YOffset - (i * $YSpacing), 0])
        {
          cylinder(d = 2, h = 30, center = true);
            translate([-$XHoleSpacing, 0, 0])
              cylinder(d = 2, h = 30, center = true);
        }
      }
    }
  }
  
}

module PulleyMountHoles()
{
  translate([10, 0, (16 / 2) - 3])
    rotate(90, [1, 0, 0])
      cylinder(d = 2, h = 50, $fn = 30, center = true);
  translate([-23, 0, (16 / 2) - 3])
    rotate(90, [1, 0, 0])
      cylinder(d = 2, h = 50, $fn = 30, center = true);
}

module MisumiLiftSlideDrillTemplate1()
{
  difference()
  {
    union()
    {
      difference()
      {
        union()
        {
          //Main body
          translate([0, 0, 0])
            RoundedBlock($XDim = 53, $YDim = 28, $ZDim = 17, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
        }
        //Double slide coupling
        translate([- 4, 0, 0])    
          cube([60 - 8, 20.5, 15.4], center = true);
        translate([- 4, 0, 0])    
          cube([60 - 8, 19.5, 20], center = true);
        //Pulley mount holes
        PulleyMountHoles();
      }
    }
  }
}

module MisumiLiftSlideInnerFirst($Multi = false, $Bottom = 0)
{
  difference()
  {
    union()
    {
      difference()
      {
        union()
        {
          //Main body
          translate([2.75, 0, 0])
            RoundedBlock($XDim = 65.5, $YDim = 28, $ZDim = 17, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
          //Pulley block
          translate([30 - 5, -($BearingHeight / 2) - 14, 2])
            RoundedBlock($XDim = 21, $YDim = $BearingHeight + 6, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
        }
        //Double slide coupling
        translate([- 4, 0, 0])    
          cube([60 - 8, 20.5, 15.4], center = true);
        translate([- 4, 0, 0])    
          cube([60 - 8, 19.5, 20], center = true);
        //Inner carrier clearence
        translate([0, -13/2, -9])
          cube([40, 13, 8]);
        //Pulley opening
        translate([30 - 5, -($BearingHeight / 2) - 14.5, 2])
          cube([13, $BearingHeight, 10], center = true);
        //Slide mount holes
        mirror([0, 0, $Bottom])
        {
          translate([10, 0, (16 / 2) - 3])
            rotate(90, [1, 0, 0])
              cylinder(d = 3, h = 50, $fn = 30, center = true);
          translate([-23, 0, (16 / 2) - 3])
            rotate(90, [1, 0, 0])
              cylinder(d = 3, h = 50, $fn = 30, center = true);
        }
      }
      //Alignment guide
      difference()
      {
        translate([15.3875, 0, 0])
          rotate(90, [0, 1, 0])
            cylinder(d = 7.3, h = 20.1);
        translate([15.4, 0, 0])
          cube([13.2, 9, 3], center = true);
      }
    }
    //Bearing shaft
    translate([30 - 5, 0, 2])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50 + $BearingHeight, $fn = 30, center = true);
    
    if ($Multi)
    {
      mirror([0, 0, $Bottom])
        translate([-40, -20, -10 - (17 / 2) + 2.5])
          cube([80, 40, 10]);
    }
  }
  if ($ShowBearingOutlines)
  {
    //Bearing
    translate([25, -19.5, 2])
      rotate(90, [1, 0, 0])
          cylinder(d = $BearingOutlineD, h = 8, $fn = 30, center = true);
  }
}

/*
module MisumiLiftSlideSpringMount($MountD = 2.9)
{
  $fn = 20;
  $WidthAdder = 10;
  difference()
  {
    union()
    {
      //Main block
      translate([90 / 2, 28 / 2 , (8 - $WidthAdder) / 2])
        RoundedBlock($XDim = 90, $YDim = 28, $ZDim = 8 + $WidthAdder, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Attach/pulley block
      translate([21 / 2, -($BearingHeight / 2), (8 + 6.5) / 2])
        RoundedBlock($XDim = 21, $YDim = $BearingHeight + 6, $ZDim = 8 + 6.5, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Spring blocker
      translate([24, -(8 - 6) / 2, - 6 + ($WidthAdder) / 2])
        RoundedBlock($XDim = 10, $YDim = 8 + 6, $ZDim = 8 + $WidthAdder, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([0, (28 - 20.2) / 2, 7])
      cube([90, 20.2, 10]);
    //Bearing opening
    translate([21/2, -($BearingHeight / 2) - 0.5, 19/2])
      cube([13, $BearingHeight, 20], center = true);
    //Bearing shaft
    translate([21 / 2, 32, 5])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Bearing cord guide
    translate([(21/2) + 9, -3, 13.5])
      cube([10, 2, 2], center = true);
    //Spring mount holes
    translate([35 + (10 / 2) + 45, 32, -6])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    translate([35 + (10 / 2) + 45, 32, -0.5])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Spring cord opening2
    translate([25, -5, -1])
      cube([20, 10, 2], center = true);
    translate([25, -5, -6])
      cube([20, 10, 2], center = true);
    //Slider attach hole options
    //Pulley block flush with top
    translate([14.5, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Pulley flush with top
    translate([14.5 + 12, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Full travel clearance
    translate([14.5 + 12 + 23, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Return clearance location 1
    translate([14.5 + 12 + 8, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Return clearance location 2
    translate([14.5 + 12 + 15, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
  }
}
*/

module MisumiLiftSlideBelt($MountD = 2.9)
{
  $fn = 20;
  $WidthAdder = 10;
  difference()
  {
    union()
    {
      //Main block
      translate([40 / 2, 28 / 2 , (8 - $WidthAdder) / 2])
        RoundedBlock($XDim = 40, $YDim = 28, $ZDim = 8 + $WidthAdder, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Belt clamp
      translate([18, -$BearingHeight / 2, - 6 + ($WidthAdder) / 2])
        RoundedBlock($XDim = 30, $YDim = $BearingHeight + 6, $ZDim = 8 + $WidthAdder, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([0, (28 - 20.2) / 2, 7])
      cube([90, 20.2, 10]);
    //Belt clamp openings
    translate([30, -($BearingHeight + 4) / 2, -4])
      cube([60, $BearingHeight + 4, 2], center = true);
    translate([30, -($BearingHeight + 4) / 2, 2])
      cube([60, $BearingHeight + 4, 2], center = true);
    //Belt clamp bolt holes
    translate([36, 0, 2])
      rotate(-90, [1, 0, 0])
        cylinder(d = 2.7, h = 25);
    translate([36, 0, -4])
      rotate(-90, [1, 0, 0])
        cylinder(d = 2.7, h = 25);
    //Slider attach hole options
    //Pulley block flush with top
    translate([14.5, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
    //Pulley flush with top
    translate([14.5 + 12, 28 / 2, -15])
      cylinder(d = $MountD, h = 30);
  }
}

module MisumiLiftSlideReturn(MountD = 2.9, Stages = 3)
{  
  difference()
  {
    union()
    {
      //Attach block
      translate([90 / 2, -28 / 2, (8 - $SpacerHeight) / 2])
        RoundedBlock($XDim = 90, $YDim = 28, $ZDim = 8 + $SpacerHeight, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Return support block
      translate([21 / 2, -28 / 2, ((8 + ($Stages * 16) + 8 - $SpacerHeight) / 2)])
        RoundedBlock($XDim = 21, $YDim = 28, $ZDim = $SpacerHeight + 8 + ($Stages * 16) + 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Pulley block
      translate([(21 / 2), ($BearingHeight  / 2), (10 / 2) + $ReturnOffset])
        RoundedBlock($XDim = 21, $YDim = $BearingHeight + 6, $ZDim = 9, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      //Pulley return block
      translate([21 / 2, $BearingHeight  / 2, (9 / 2) + (16 * $Stages) + 7])
        RoundedBlock($XDim = 21, $YDim = $BearingHeight + 6, $ZDim = 9, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([21, -(28 + 20.5) / 2, 7])
      cube([90, 20.5, 10]);
    //Bearing opening
    translate([21/2, ($BearingHeight / 2) + .5, (9/2) + $ReturnOffset])
      cube([13, $BearingHeight, 15], center = true);
    //Return bearing opening
    translate([21/2, ($BearingHeight / 2) + .5, (9/2) + (16 * $Stages) + 5])
      cube([13, $BearingHeight, 20], center = true);
    //Bearing shaft
    translate([21 / 2, 15, 5 + $ReturnOffset])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Return bearing shaft
    translate([21 / 2, 15, 5  + (16 * $Stages) + 7])
      rotate(90, [1, 0, 0])
        cylinder(d = 3, h = 50, $fn = 30);
    //Mount hole
    translate([14.5 + 12 + 8 + 1.5, -28 / 2, -1])
#      cylinder(d = $MountD, h = 50, center = true);
      
  }
  if ($ShowBearingOutlines)
  {
    translate([0, -9.5, 0])
    {
      //Bearings
      translate([21 / 2, 15, 5 + $ReturnOffset])
        rotate(90, [1, 0, 0])
          cylinder(d = $BearingOutlineD, h = 8, $fn = 30, center = true);
      //Return bearing
      translate([21 / 2, 15, 5  + (16 * $Stages) + 7])
        rotate(90, [1, 0, 0])
          cylinder(d = $BearingOutlineD, h = 8, $fn = 30, center = true);
    }
  }
}

module MisumiLiftSlideOuterFirst($MountD = 2.9, $DoBearing)
{
  difference()
  {
    union()
    {
      translate([90 / 2, 28 / 2, ((8 - $SpacerHeight) / 2)])
        RoundedBlock($XDim = 90, $YDim = 28, $ZDim = 8 + $SpacerHeight, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
      if ($DoBearing)
        translate([21 / 2, -$BearingHeight / 2, 8 / 2])
          RoundedBlock($XDim = 21, $YDim = $BearingHeight + 6, $ZDim = 8, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([0, (28 - 20.2) / 2, 7])
      cube([90, 20.2, 10]);
    //Bearing opening
    translate([21/2, -($BearingHeight / 2) - 0.5, 9/2])
      cube([13, $BearingHeight, 10], center = true);
    if ($DoBearing)
      translate([21 / 2, 32, 5])
        rotate(90, [1, 0, 0])
          cylinder(d = 3, h = 50, $fn = 30);
    //Slider attach hole options
    //Pulley block flush with top
    translate([14.5, 28 / 2, -1])
      cylinder(d = $MountD, h = 20, center = true);
    //Pulley flush with top
    translate([14.5 + 12, 28 / 2, -1])
      cylinder(d = $MountD, h = 20, center = true);
    //Full travel clearance
    translate([14.5 + 12 + 23, 28 / 2, -1])
      cylinder(d = $MountD, h = 20, center = true);
    //Return clearance location 1
    translate([14.5 + 12 + 8, 28 / 2, -1])
      cylinder(d = $MountD, h = 20, center = true);
    //Return clearance location 2
    translate([14.5 + 12 + 15, 28 / 2, -1])
      cylinder(d = $MountD, h = 20, center = true);
  }
  if ($ShowBearingOutlines && $DoBearing)
  {
    translate([0, -9.5, 0])
    {
      //Bearing
      translate([21 / 2, 4, 5])
        rotate(90, [1, 0, 0])
            cylinder(d = $BearingOutlineD, h = 8, $fn = 30, center = true);
    }
  }
}

module PrintMisumiSliderFirstStage($Sliders = 3)
{
  //Outer first stage top bearing mount
  translate([-40, -5, $SpacerHeight])
    MisumiLiftSlideOuterFirst($DoBearing = true);
//  //Return
//  translate([65 + $BearingHeight, 32, 28])
//    rotate(180, [0, 0, 1])
//      rotate(90, [1, 0, 0])
//        MisumiLiftSlideReturn($MountD = 2.9, $Stages = $Sliders);
//  //Spacer
//  translate([-50, 0, $SpacerHeight])
//    rotate(90, [0, 0, 1])
//    MisumiLiftSlideOuterFirst($DoBearing = false);
}

module PrintMisumiSliderSet($Sliders = 3)
{
  //******************************************
  //******************************************
  //Misumi slider kits
  //******************************************
  //******************************************
  //Outer first stage top bearing mount
  translate([-40, -5, $SpacerHeight])
    MisumiLiftSlideOuterFirst($DoBearing = true);
  //Return
  translate([65 + $BearingHeight, 32, 28])
    rotate(180, [0, 0, 1])
      rotate(90, [1, 0, 0])
        MisumiLiftSlideReturn($MountD = 2.9, $Stages = $Sliders);
  //Spacer
  translate([-50, 0, $SpacerHeight])
    rotate(90, [0, 0, 1])
    MisumiLiftSlideOuterFirst($DoBearing = false);
  
  if ($Sliders > 1)
  {
    for ($i = [1:$Sliders - 1])
    {
      translate([10, ($i * (37 + $BearingHeight)) + 23, 8.5])
        rotate(180, [1, 0, 0])
          MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 0);//Top inner pulley
      translate([-10, ($i * (37 + $BearingHeight)) + 13, 6.012])
          rotate(180, [0, 1, 0])
            MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 1);//Bottom inner pulley
    }
  }
  translate([63, 55 + ($Sliders * 16), 28])
    rotate(-90, [0, 1, 0])
      rotate(90, [0, 0, 1])
        MisumiLiftSlideBelt($MountD = 2.9);
}

module MisumiRailSet(support1 = true, length = 300, stages = 2, position = 100, offset = 0, motorposition, channelholes, offsetholes, includespacers = true, dopulleys = true)
{
  hoffset = 0;//support1 ? 15 : 0;
  railoffset = includespacers ? 15 : 0;
  //Return style 0 = Return cord lines up with edge of outer slide
  //Return style 1 = Return cord lines up with pull up pulley
//  returnextension = (returnstyle == 0) ? ((15/2) + (stages * 16) + (hoffset * (stages - 1)) - 3) :
//                                         ((15/2) + (stages * 16) + (hoffset * (stages - 1)) - 13);
  StageOffset = position / stages;
 
  for (i = [0:(stages - 1)])
  {
    translate([0, (i * (16.1 + railoffset)) + hoffset - 15, (i * offset * 2) + (i * StageOffset)])
    {
      if ((support1 && (i == 0)) || includespacers)
        translate([0, (15/2), 70])
          Extrusion15mm(length = length - 100);

      if (stages == 1)
        translate([0, 15 , offset])
          MisumiSlide(length = length, position = StageOffset, showupper = 2, showlower = 3, showpulley = 0, offset = offset);//Single stage so only need single upper and lower belt block
      else if (i == 0)//First, fixed stage
        translate([0, 15 , offset])
          MisumiSlide(length = length, position = StageOffset, showupper = 1, showlower = 0, showpulley = 0, offset = offset);//Only upper pulley, outside type
      else if (i == stages - 1)//Last, outer stage
        translate([0, 15 , offset])
          MisumiSlide(length = length, position = StageOffset, showupper = 2, showlower = 2, showpulley = 0, offset = offset);//Upper outside pulley and lower belt block
      else
        translate([0, 15 , offset])
          MisumiSlide(length = length, position = StageOffset, showupper = 2, showlower = 1, showpulley = 0, offset = offset);//Upper and lower pulleys, inside types
    }
  }

  if(dopulleys)
  {
    color("RoyalBlue")
    translate([14, -7.1, -21.1])
      rotate(-90, [0, 0, 1])
        rotate(-90, [0, 1, 0])
          MisumiLiftSlideReturn($MountD = 2.9, $Stages = stages);
  }
  
  //Motor mount
  if (motorposition != -1)
  {
    translate([5, -33, motorposition])
      rotate(-90, [0, 1,0])
        MotorAndFrame(ChannelHoles = channelholes, Rx = -1, Ry = 0, Rz = -1, OffsetHoles = offsetholes);
  }
}

module MisumiSlideRail(length = 300, domountholes = true)
{
  difference()
  {
    translate([-10, 0, 0])
      cube([20, 7.2, length]);
    translate([-8, 2.01, -0.5])
      cube([16, 5.2, length + 1]);
    rotate(-90, [1, 0, 0])
    {
      translate([0, -15, -0.1])
        cylinder(d = 3.1, h = 10);
      translate([0, -length + 15, -0.1])
        cylinder(d = 3.1, h = 10);
      translate([0, -length / 2, -0.1])
        cylinder(d = 3.1, h = 10);
    }
    translate([0, 10.1, 10 + 11.9])
      rotate(90, [0, 1, 0])
        rotate(90, [1, 0, 0])
          PulleyMountHoles();
    translate([0, 10.1, length - 23 - 11.9])
      rotate(90, [0, 1, 0])
        rotate(90, [1, 0, 0])
          PulleyMountHoles();
  }
}
 
module Extrusion15mm(length = 300)
{
  color([0.95, 0.9, 0.95])
    translate([0, 0, length / 2])
      cube([15, 15, length], center = true);
  color([.8, .8, .8])
  {
    translate([0, 0, length / 2])
      cube([3.001, 15.001, length + 0.001], center = true);
    translate([0, 0, length / 2])
      cube([15.001, 3.001, length + 0.001], center = true);
  }
}

module TimingBeltPulleyGoBilda14mm()
{
  $fn = 200;
  
  //Inner pitch diameter
  cylinder(d = 38.2, h = 7, center = true);
  //Belt outer diameter
  color([0.2, 0.2, 0.2, 0.5])
    cylinder(d = 38.2 + 0.6 + 0.6, h = 7, center = true);
  //Flange outer diameters
  color("CadetBlue")
  {
    translate([0, 0, (7 + 0.75) / 2])
      cylinder(d = 40, h = 1.5 / 2, center = true);
    translate([0, 0, -(7 + 0.75) / 2])
      cylinder(d = 40, h = 1.5 / 2, center = true);
      }
}

module MotorAndFrame(ChannelHoles, Rx, Ry, Rz, OffsetHoles = 0)
{
  //Motor
  cylinder(d = 36, h = 117);
  //Motor shaft
  translate([0, 0, -20])
    color("silver")
      cylinder(d = 8, h = 20, $fn = 6);
  //Mounting plate
  translate([0, 0, -4])
    rotate(180, [1, 0, 0])
      MotorPlate($Type = 2);
  //Support plate
  translate([0, 0, -4 + 15 + $PlateThickness])
    rotate(180, [1, 0, 0])
      MotorPlate($Type = 2);
  //Pulley
  translate([0, 0, -10 - $PlateThickness - 1.4])
    TimingBeltPulleyGoBilda14mm();
}

module MisumiSlide(length, position, showupper = 0, showlower = 0, showpulley = 0, offset, mountholes = true)
{
  color("silver")
  {
    MisumiSlideRail(length = length, domountholes = mountholes);
    translate([0, 16, position])
      mirror([0, 1, 0])
        MisumiSlideRail(length = length, domountholes = mountholes);
  }
  color("darkgray")
    translate([-8, 2, position / 2])
      cube([16, 12, length]);   

  if (showupper == 1)
  {
    color("RoyalBlue")
    translate([14.0, -7.1, length])
      rotate(-90, [1, 0, 0])
        rotate(90, [0, 0, 1])
          MisumiLiftSlideOuterFirst($DoBearing = true);
  }
  
  if (showupper == 2)
  {
    color("RoyalBlue")
    translate([0, 0, length - 22])
      rotate(90, [1, 0, 0])
        rotate(90, [0, 0, 1])
          MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 0);//Upper inner pulley
  }

  if ((showlower == 1) || (showlower == 2))
  {
    color("RoyalBlue")
      translate([0, 0, 22])
        rotate(90, [0, 1, 0])
          rotate(-90, [1, 0, 0])
            MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 1);//Bottom inner pulley
  }
  
  if ((showlower == 2) || (showlower == 3))
  {
    color("RoyalBlue")
      translate([14, 23.1, position])
        rotate(-90, [0, 1, 0])
          rotate(90, [1, 0, 0])
            MisumiLiftSlideBelt($MountD = 2.9);
  }
}


module ElevatorMotor()
{

    difference()
    {
        hull()
        {
        cylinder(d = 40, h = 1);
        translate([-14, -22.6, 0])
            cylinder(d = 13, h = 1);
        translate([14, -22.6, 0])
            cylinder(d = 13, h = 1);
        }
      //Motor shaft opening
      cylinder(d = 10, h = $PlateThickness + 0.01, center = true);
      //Motor mount holes
      translate([8, 8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      translate([-8, 8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      translate([8, -8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      translate([-8, -8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      //Bearing holes
    translate([-14, -22.6, 0])
        cylinder(d = 4.1, h = 10, center = true);
    translate([14, -22.6, 0])
        cylinder(d = 4.1, h = 10, center = true);
        
   }
}

module ElevatorMotorGuide()
{
    $D = 16;
    difference()
    {
        hull()
        {
        translate([-4, -22.6, 0])
            cylinder(d = $D, h = 1);
        translate([4, -22.6, 0])
            cylinder(d = $D, h = 1);
         translate([-14, -22.6, 0])
            cylinder(d = $D, h = 1);
        translate([14, -22.6, 0])
            cylinder(d = $D, h = 1);
        }
    //Pulley clearance
#      cylinder(d = 35, h = $PlateThickness + 0.01, center = true);
      //Bearing holes
    translate([-14, -22.6, 0])
        cylinder(d = 4.1, h = 10, center = true);
    translate([14, -22.6, 0])
        cylinder(d = 4.1, h = 10, center = true);
        
   }
}

module MotorPlate()
{
  $CornerD = 5;
  $MotorEffectiveD = 41;
  $InterfaceWidth = 14;
  
  $PulleyX1 = 14;
  $PulleyY1 = 22.6;
  $PulleyX2 = 14 + 7;
  $PulleyY2 = 22.6 - 6.3;
  
  $MType = $Type;
  
  difference()
  {
    if ($MType == 3)
      hull()
      {
        cylinder(d = 40, h = $PlateThickness, center = true);
        translate([28, -20, 0])
          cylinder(d = 15, h = $PlateThickness, center = true);
      }    
    else if (($MType == 2) || ($MType == 4))
      hull()
      {
        cylinder(d = 40, h = $PlateThickness, center = true);
        translate([28, -20, 0])
          cylinder(d = 15, h = $PlateThickness, center = true);
        translate([-28, -20, 0])
          cylinder(d = 15, h = $PlateThickness, center = true);
      }
    else if ($MType == 1)
      hull()
      {
        cylinder(d = 35, h = $PlateThickness, center = true);
        translate([0, -($MotorEffectiveD + $InterfaceWidth) / 2, 0])
          RoundedPlate($XDim = 70, $YDim = $InterfaceWidth, $ZDim = $PlateThickness, $D1 = $CornerD, $D2 = $CornerD, $D3 = $CornerD, $D4 = $CornerD);
      }
    else if ($MType == 0)
      hull()
      {
        cylinder(d = 50, h = $PlateThickness, center = true);
        translate([0, -($MotorEffectiveD + $InterfaceWidth) / 2, 0])
          RoundedPlate($XDim = 70, $YDim = $InterfaceWidth, $ZDim = $PlateThickness, $D1 = $CornerD, $D2 = $CornerD, $D3 = $CornerD, $D4 = $CornerD);
      }
      
    //8mm generic M4 mount holes
    translate([28, (-($MotorEffectiveD + $InterfaceWidth) / 2) + 4, 0])
      cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
    translate([-28, (-($MotorEffectiveD + $InterfaceWidth) / 2) + 4, 0])
      cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
    translate([4, (-($MotorEffectiveD + $InterfaceWidth) / 2) + 4, 0])
      cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
    translate([-4, (-($MotorEffectiveD + $InterfaceWidth) / 2) + 4, 0])
      cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
    
    if (($MType == 0) || ($MType == 1))
    {
      //Rail mount holes
      translate([-24, (-($MotorEffectiveD + $InterfaceWidth) / 2) - 1, 0])
        cylinder(d = 3.1, h = $PlateThickness + 0.01, center = true);
      translate([24, (-($MotorEffectiveD + $InterfaceWidth) / 2) - 1, 0])
        cylinder(d = 3.1, h = $PlateThickness + 0.01, center = true);
      translate([0, (-($MotorEffectiveD + $InterfaceWidth) / 2) - 1, 0])
        cylinder(d = 3.1, h = $PlateThickness + 0.01, center = true);
    }
    
    if ($MType != 0)
    {
      //Retained bearing holes position 1
      translate([$PulleyY1, -$PulleyX1, 0])
        cylinder(d = 3.1, h = 12, center = true);
      translate([-$PulleyY1, -$PulleyX1, 0])
        cylinder(d = 3.1, h = 12, center = true);

      //Retained bearing holes position 2
      translate([$PulleyY2, -$PulleyX2, 0])
        cylinder(d = 3.1, h = 12, center = true);
      translate([-$PulleyY2, -$PulleyX2, 0])
        cylinder(d = 3.1, h = 12, center = true);
    }
      
    if ($MType == 0)
      //Motor opening
      cylinder(d = 37, h = $PlateThickness + 0.01, center = true);
    else if (($MType == 1) || ($MType == 2) || ($MType == 3))
    {
      //Motor shaft opening
      cylinder(d = 10, h = $PlateThickness + 0.01, center = true);
      //Motor mount holes
      translate([8, 8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      translate([-8, 8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      translate([8, -8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      translate([-8, -8, 0])
        cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      //Motor mount holes 45 degrees rotated
      rotate(45, [0, 0, 1])
      {
        translate([8, 8, 0])
          cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
        translate([-8, 8, 0])
          cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
        translate([8, -8, 0])
          cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
        translate([-8, -8, 0])
          cylinder(d = 4.1, h = $PlateThickness + 0.01, center = true);
      }
    }
  }
  if ($ShowBearingOutlines)
  {
    color("SlateGray")
    {
      //Pulley belt guides set 1
      translate([$PulleyY1, -$PulleyX1, 3.5])
        cylinder(d = 13, h = 12);
      translate([-$PulleyY1, -$PulleyX1, 3.5])
        cylinder(d = 13, h = 12);
    }
    color("DarkGray")
    {
      //Pulley belt guides set 2
      translate([$PulleyY2, -$PulleyX2, 3.5])
        cylinder(d = 13, h = 12);
      translate([-$PulleyY2, -$PulleyX2, 3.5])
        cylinder(d = 13, h = 12);
        }
  }
}

module MisumiLiftSlide3mmShim()
{
  $MountD = 3;
  difference()
  {
    union()
    {
      translate([90 / 2, 28 / 2, ((8 - $SpacerHeight) / 2)])
        RoundedBlock($XDim = 90, $YDim = 28, $ZDim = 3, $D1 = 2, $D2 = 2, $D3 = 2, $D4 = 2, $D5 = 2, $D6 = 2, $D7 = 2, $D8 = 2);
    }
    //Slider alignment groove
    translate([0, (28 - 20.2) / 2, 7])
      cube([90, 20.2, 10]);
    //Bearing opening
    translate([21/2, -($BearingHeight / 2) - 0.5, 9/2])
      cube([13, $BearingHeight, 10], center = true);
    //Slider attach hole options
    //Pulley block flush with top
    translate([14.5, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Pulley flush with top
    translate([14.5 + 12, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Full travel clearance
    translate([14.5 + 12 + 23, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Return clearance location 1
    translate([14.5 + 12 + 8, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
    //Return clearance location 2
    translate([14.5 + 12 + 15, 28 / 2, -1])
      cylinder(d = $MountD, h = 20);
  }
}


/*
module PulleyPlate(TopMount = false)
{
  $CornerD = 8;
  $BaseHeight = 70;
  $BaseWidth = 12;
  $MountSpacing = 33;
  $BearingMountD = 4.1;
  $PlateMountD = 2.6;
  
  $MountOffset = TopMount ? 1.5:-1.5;
  $PulleyOffset = TopMount ? -8:-1.5;
  
  rotate(90, [0, 1, 0])
  {
    difference()
    {      
      union()
      {
        translate([(-$BaseHeight / 2) + 5, $MountOffset, 0])
          RoundedPlate($XDim = $BaseHeight, $YDim = $BaseWidth, $ZDim = $PlateThickness, $D1 = $CornerD, $D2 = $CornerD, $D3 = $CornerD, $D4 = $CornerD);
        //Angled bearing offset
        hull()
        {
          translate([-$BaseHeight + 9, $MountOffset, 0])
            cylinder(d = $BaseWidth, h = $PlateThickness, center = true);
          translate([-$BaseHeight - 3, $PulleyOffset, 0])
            cylinder(d = $BaseWidth, h = $PlateThickness, center = true);
        }
      }
      //Pulley plate mount holes
      //Longest extension holes
      cylinder(d = $PlateMountD, h = 5, center = true);
      translate([-$MountSpacing, 0, 0])
        cylinder(d = $PlateMountD, h = 5, center = true);
      //Partial extension holes
      translate([-11, 0, 0])
      {
        cylinder(d = $PlateMountD, h = 5, center = true);
        translate([-$MountSpacing, 0, 0])
          cylinder(d = $PlateMountD, h = 5, center = true);
      }
      //Compact extension holes
      translate([-22, 0, 0])
      {
        cylinder(d = $PlateMountD, h = 5, center = true);
        translate([-$MountSpacing, 0, 0])
          cylinder(d = $PlateMountD, h = 5, center = true);
      }
      //Bearing mount hole
      translate([-$BaseHeight - 3, $PulleyOffset, 0])
        cylinder(d = $BearingMountD, h = $PlateThickness, center = true);

    }
      //Bearing
      color([0.6, 0.6, 0.8])
      translate([-$BaseHeight - 3, $PulleyOffset, -5.7])
        cylinder(d = 8, h = 8, center = true);
  }
}
*/

//******************************************
//******************************************
//mirror([1, 0, 0])
//  MisumiLiftSlideOuterFirst($DoBearing = true);
//  MisumiLiftSlideOuterFirst($DoBearing = false);
//*MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 0);//Top inner pulley
//*MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 1);//Bottom inner pulley
//******************************************
//******************************************
//PrintMisumiSliderSet($Sliders = 3);
//PrintMisumiSliderFirstStage($Sliders = 3);

//MisumiLiftSlideInnerFirst($Multi = true, $Bottom = 1);
//*MisumiLiftSlideBelt($MountD = 2.9);
//*MisumiLiftSlideReturn($Stages = 3, $MountD = 2.9);
//MisumiLiftSlideDrillTemplate1();
//MisumiLiftSlideDrillTemplate2($Stages = 1);
// Broken MisumiRailSet(support1 = true, length = 300, stages = 2, position = 100, offset = 0, motorposition = 40, channelholes = 3, offsetholes = 10);
//projection()
//MotorPlate($Type = 4);


//MisumiSlide(length = 300, position = 0, showupper = true, showlower = true, pulleyextension = 0, showpulley = true, offset = 0);
//MisumiRailSet(support1 = false, length = 300, stages = 3, position = 100, offset = 0, motorposition = 100, channelholes = 0, offsetholes = 0, includespacers = false);
//MisumiRailSet(support1 = false, length = 300, stages = 1, position = 100, offset = 0, motorposition = 150, channelholes = 0, offsetholes = 0, includespacers = false);

//MisumiLiftSlide3mmShim();

projection()
//    ElevatorMotor();
ElevatorMotorGuide();
