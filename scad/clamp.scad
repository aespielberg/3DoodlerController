ext_length = 9;
finger_width = 9.5;
finger_depth = 13.5;
space = 0.5;

height = 20; //personal choice
padding = 5;
lower_diam = 28.5;
upper_diam = 38.8;

indent_1 = 22.5;
indent_2 = 123.25;

total_length = 124;

//TODO: 15 mm extrusion

clamp();

module base(){
    difference(){
        cube([ext_length + finger_width + padding,finger_depth + padding,height]);
        translate([padding + finger_width/2, padding/2, 0]) cube([finger_width + space, finger_depth + space, height]);
    }
}

module extension(){
    difference(){
        cube([ext_length + finger_width + padding,finger_depth + padding, total_length + padding]);   
        translate([padding + finger_width/2, padding/2, 0]) cube([finger_width + space, finger_depth + space, total_length - height]);
    }
}

module lower_ring(){
    difference(){
        cylinder(h=padding, r=space/2 + lower_diam/2 + padding*2);
        cylinder(h=padding, r=space/2 + lower_diam/2); 
        translate([(space/2 + lower_diam/2 + padding/2), 0, padding/2]) cube( [space + lower_diam + padding, space + lower_diam + padding*5, padding], true);   
    }
}

module upper_ring(){
    difference(){
        cylinder(h=padding, r=space/2 + upper_diam/2 + padding*2);
        cylinder(h=padding, r=space/2 + upper_diam/2);
        translate([(space/2 + upper_diam/2 + padding/2), 0, padding/2]) cube( [space + upper_diam + padding, space + upper_diam + padding*5, padding], true); 
    }
}

module connector(){
    cube([(upper_diam - lower_diam)/2, finger_depth + padding, padding]);
}


module clamp(){
        union(){
            base();
            translate([0, 0, height]) extension();
            translate([ext_length + finger_width + padding, 0.0, 0.0]) translate([0.0, 0.0, height + indent_1]) connector();
            translate([(upper_diam - lower_diam)/2, 0, 0]) translate([space/2 + lower_diam/2, 0, 0]) translate([ext_length + finger_width + padding, (finger_depth + padding)/2.0, 0]) translate([0.0, 0.0, height + indent_1]) lower_ring();
            translate([space/2 + upper_diam/2, 0, 0]) translate([ext_length + finger_width + padding, (finger_depth + padding)/2.0, 0]) translate([0.0, 0.0, height + indent_2]) upper_ring();
        }

    
}