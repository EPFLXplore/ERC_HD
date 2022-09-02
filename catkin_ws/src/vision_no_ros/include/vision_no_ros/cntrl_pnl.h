#ifndef CNTRL_PNL_H
#define CNTRL_PNL_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/opencv.hpp>


#define Dist1 71
#define Dist2 42
#define Dist3 62
#define Wdth1 50  //this is also the width of the individual objects
#define Wdth2 32 
#define Switch_width 25

namespace cntrl_pnl {
    
    struct Position {  // Keeps x,y pos of the center of the element
        float x_coor;
        float y_coor; 
    };

    struct Object { // Structure for Switches, Buttons, interactive elements
    
        char name[6];
        Position pos;
        int id;      
    };

    struct ArTag { //Structure for ArTags
    
        char name[6];
        Position pos;
        float width;  // this will be neede to modify hte estimateposesinglemarkere function's ar tag dimension parameter !! some ar tags might have different sizes !!
        int id;
    };

    struct PanelA {
        char name[6];
        ArTag artg1; //removed artag 2, it no longer exists
        Object switchMain, switch1, switch2, switch3, switch4, switch5, switch6, switch7, switch8, switch9; //added main switch

    };

    struct PanelB1 {
        char name[6];
        ArTag artg2, artg3;
        Object outlet, button, emagLock;

    };

    struct PanelB2 {
        char name[6];
        ArTag artg4;
        Object ethernet;

    };

    struct ControlPanel{

        PanelA panelA;
        PanelB1 panelB1;
        PanelB2 panelB2;


    };

    void setup_control_panel(ControlPanel& control_panel) {

    // Setup of Panel A

        PanelA PNLA;      
        strcpy(PNLA.name,"PNLA");

        strcpy(PNLA.artg1.name, "ARTG1");
        PNLA.artg1.pos.x_coor= -Dist2;   //modified panel one for new rules
        PNLA.artg1.pos.y_coor= Dist1;    
        PNLA.artg1.width=Wdth1;
        PNLA.artg1.id = 2;//this should be 2              // need to put the right Id's!! maybe make a setup function with a camera so we can create the panel with just a picture
 
        /*//this Ar tag no longer exists
        strcpy(PNLA.artg2.name, "ARTG2"); 
        PNLA.artg2.pos.x_coor= -Dist2;
        PNLA.artg2.pos.y_coor= 0;
        PNLA.artg2.width=Wdth1;
        PNLA.artg2.id = 2;
        */
        //distances are taken from the center of each panel
        strcpy(PNLA.switchMain.name, "SWTCMain");
        PNLA.switchMain.pos.x_coor= Dist2-Switch_width;
        PNLA.switchMain.pos.y_coor= Dist1;
        PNLA.switchMain.id = 1;

        strcpy(PNLA.switch1.name, "SWTC1");
        PNLA.switch1.pos.x_coor= Dist2+Switch_width; 
        PNLA.switch1.pos.y_coor= Dist1;
        PNLA.switch1.id = 2;

        strcpy(PNLA.switch2.name, "SWTC2");
        PNLA.switch2.pos.x_coor= -Dist2-Switch_width;
        PNLA.switch2.pos.y_coor= 0;
        PNLA.switch2.id = 3;

        strcpy(PNLA.switch3.name, "SWTC3");
        PNLA.switch3.pos.x_coor= -Dist2+Switch_width;
        PNLA.switch3.pos.y_coor= 0;
        PNLA.switch3.id = 4;

        strcpy(PNLA.switch4.name, "SWTC4");
        PNLA.switch4.pos.x_coor= Dist2-Switch_width;
        PNLA.switch4.pos.y_coor= 0;
        PNLA.switch4.id = 5;

        strcpy(PNLA.switch4.name, "SWTC5");
        PNLA.switch5.pos.x_coor= Dist2+Switch_width;
        PNLA.switch5.pos.y_coor= 0;
        PNLA.switch5.id = 6;

        strcpy(PNLA.switch4.name, "SWTC6");
        PNLA.switch6.pos.x_coor= -Dist2-Switch_width;
        PNLA.switch6.pos.y_coor= -Dist1;
        PNLA.switch6.id = 7;

        strcpy(PNLA.switch4.name, "SWTC7");
        PNLA.switch7.pos.x_coor= -Dist2+Switch_width;
        PNLA.switch7.pos.y_coor= -Dist1;
        PNLA.switch7.id = 8;

        strcpy(PNLA.switch4.name, "SWTC8");
        PNLA.switch8.pos.x_coor= Dist2-Switch_width;
        PNLA.switch8.pos.y_coor= -Dist1;
        PNLA.switch8.id = 9;

        strcpy(PNLA.switch4.name, "SWTC9");
        PNLA.switch9.pos.x_coor= Dist2+Switch_width;
        PNLA.switch9.pos.y_coor= -Dist1;
        PNLA.switch9.id = 10;

        control_panel.panelA = PNLA;

    // Setup of Panel B1

        PanelB1 PNLB1;
        strcpy(PNLB1.name, "PNLB1");

        strcpy(PNLB1.artg2.name, "ARTG2");
        PNLB1.artg2.pos.x_coor= Dist3;
        PNLB1.artg2.pos.y_coor= Dist1/2.0;
        PNLB1.artg2.width=Wdth2;
        PNLB1.artg2.id = 1;

        strcpy(PNLB1.artg3.name, "ARTG3");
        PNLB1.artg3.pos.x_coor= Dist3;
        PNLB1.artg3.pos.y_coor= -Dist1/2.0;
        PNLB1.artg3.width=Wdth2;
        PNLB1.artg3.id = 0;

        strcpy(PNLB1.outlet.name, "OUTLET");
        PNLB1.outlet.pos.x_coor= 0;
        PNLB1.outlet.pos.y_coor= -Dist1/2.0;
        PNLB1.outlet.id = 12;

        strcpy(PNLB1.button.name, "BUTTON");
        PNLB1.button.pos.x_coor= 0;
        PNLB1.button.pos.y_coor= Dist1/2.0;
        PNLB1.button.id = 11;

        strcpy(PNLB1.emagLock.name, "EMAGLOCK");
        PNLB1.emagLock.pos.x_coor= -123.5;
        PNLB1.emagLock.pos.y_coor= -Dist1/2.0+6.0;
        PNLB1.emagLock.id = 13;



        control_panel.panelB1 = PNLB1;
    // Setup of Panel B2

        PanelB2 PNLB2;

        strcpy(PNLB2.name, "PNLB2");

        strcpy(PNLB2.artg4.name, "ARTG4");
        PNLB2.artg4.pos.x_coor= 0;
        PNLB2.artg4.pos.y_coor= -Dist1/2.0;
        PNLB2.artg4.width=Wdth1;
        PNLB2.artg4.id=3;

        strcpy(PNLB2.ethernet.name, "ETHRN");
        PNLB2.ethernet.pos.x_coor= 0;
        PNLB2.ethernet.pos.y_coor= Dist1/2.0;
        PNLB2.ethernet.id = 14;

        control_panel.panelB2 = PNLB2;
    // Verification prints for Panel A 
        
        printf("%s \n%s \n%f \n%f \n%f\n\n", PNLA.name, PNLA.artg1.name, PNLA.artg1.pos.x_coor, PNLA.artg1.pos.y_coor, PNLA.artg1.width);
       // printf("%s \n%s \n%f \n%f \n%f\n\n", PNLA.name, PNLA.artg2.name, PNLA.artg2.pos.x_coor, PNLA.artg2.pos.y_coor, PNLA.artg2.width);
        printf("%s \n%s \n%f \n%f \n%d\n\n", PNLA.name, PNLA.switchMain.name, PNLA.switchMain.pos.x_coor, PNLA.switchMain.pos.y_coor,PNLA.switchMain.id);
        printf("%s \n%s \n%f \n%f\n\n", PNLA.name, PNLA.switch1.name, PNLA.switch1.pos.x_coor, PNLA.switch1.pos.y_coor);
        printf("%s \n%s \n%f \n%f\n\n", PNLA.name, PNLA.switch2.name, PNLA.switch2.pos.x_coor, PNLA.switch2.pos.y_coor);
        printf("%s \n%s \n%f \n%f\n\n", PNLA.name, PNLA.switch3.name, PNLA.switch3.pos.x_coor, PNLA.switch3.pos.y_coor);
        printf("%s \n%s \n%f \n%f\n\n", PNLA.name, PNLA.switch4.name, PNLA.switch4.pos.x_coor, PNLA.switch4.pos.y_coor);


    // Verification prints for Panel B1

        printf("%s \n%s \n%f \n%f \n%f\n\n", PNLB1.name, PNLB1.artg2.name, PNLB1.artg2.pos.x_coor, PNLB1.artg2.pos.y_coor, PNLB1.artg2.width);
        printf("%s \n%s \n%f \n%f \n%f\n\n", PNLB1.name, PNLB1.artg3.name, PNLB1.artg3.pos.x_coor, PNLB1.artg3.pos.y_coor, PNLB1.artg3.width);
        printf("%s \n%s \n%f \n%f\n\n", PNLB1.name, PNLB1.outlet.name, PNLB1.outlet.pos.x_coor, PNLB1.outlet.pos.y_coor);
        printf("%s \n%s \n%f \n%f\n\n", PNLB1.name, PNLB1.button.name, PNLB1.button.pos.x_coor, PNLB1.button.pos.y_coor);
        printf("%s \n%s \n%f \n%f\n\n", PNLB1.name, PNLB1.emagLock.name, PNLB1.emagLock.pos.x_coor, PNLB1.emagLock.pos.y_coor);

    // Verification prints for Panel B2

        printf("%s \n%s \n%f \n%f \n%f\n\n", PNLB2.name, PNLB2.artg4.name, PNLB2.artg4.pos.x_coor, PNLB2.artg4.pos.y_coor, PNLB2.artg4.width);
        printf("%s \n%s \n%f \n%f\n\n", PNLB2.name, PNLB2.ethernet.name, PNLB2.ethernet.pos.x_coor, PNLB2.ethernet.pos.y_coor); 

    }

    /*
     function returns distance between an AT tag and a panel object, origin is set as the center of the AR tag
    */
    Position distance_from_ARtag(const ArTag& artag,const Object& object){
        Position distance;
        distance.x_coor=abs(artag.pos.x_coor-object.pos.x_coor);
        distance.y_coor=abs(artag.pos.y_coor-object.pos.y_coor);
        if (object.pos.x_coor<artag.pos.x_coor)
            distance.x_coor=-distance.x_coor;  // get the distance relative to the AR tag
        if (object.pos.y_coor<artag.pos.y_coor)
            distance.y_coor=-distance.y_coor;
        return distance;  //unit is mm
    }

}
#endif