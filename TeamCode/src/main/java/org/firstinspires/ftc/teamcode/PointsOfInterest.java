package org.firstinspires.ftc.teamcode;

import java.util.Locale;

/*
class created by Stephen Duffy and Smyan Sengupta




a class that can be used to refer to various point of interest on the field

do not edit without permission
 */
public class PointsOfInterest {
    static Point[] barcode = {//create an array of points for barcodes
            //B=blue, R=red, C=carousel side, W=warehouse side, number corresponds to level of shipping hub
            new Point(34.25,42.5,"BC1"),//barcodes on the blue carousel side
            new Point(34.25,34,"BC2"),
            new Point(34.25,25.5,"BC3"),
            new Point(34.25,90,"BW1"),//barcodes on the blue warehouse side
            new Point(34.25,81.5,"BW2"),
            new Point(34.25,73,"BW3"),
            new Point(109.75,42.5,"RC1"),//barcodes on the red carousel side
            new Point(109.75,34,"RC2"),
            new Point(109.75,25.5,"RC3"),
            new Point(109.75,90,"RW1"),//barcodes on the red warehouse side
            new Point(109.75,81.5,"RW2"),
            new Point(109.75,73,"RW3")

    };

    static Point[] hubs = { //points for centers of shipping hubs
        new Point(48,60,"BSH"), //blue shipping hub
        new Point(96,60,"RSH"), //red shipping hub
        new Point(72,120,"SSH") //shared shipping hub
    };

    static Point[] warehouses = { //points for boundaries of warehouses; used in inWareHouse, inBlueWarehouse, and inRedWarehouse functions
        new Point(43.5,100.5,"BWH"), //blue warehouse boundaries
        new Point(100.5,100.5,"RWH") //red warehouse boundaries
    };

    public static Point[] getBarcodePosition(String name){//returns an array of 1 or mode barcodes that meet the requested criteria
        String name2=name.toLowerCase();
        if(name2.equals("barcode")||name2.equals("barcodes")||name2.equals("all"))//return all the barcode postiions
            return barcode;
        if(name2.equals("barcode blue")||name2.equals("blue barcode")||name2.equals("blue barcodes")||name2.equals("barcodes blue")){//return all the barcodes on the blue side
            Point output[]= new Point[6];
            output[0]=barcode[0];
            output[1]=barcode[1];
            output[2]=barcode[2];
            output[3]=barcode[3];
            output[4]=barcode[4];
            output[5]=barcode[5];
            return output;
        }

        if(name2.equals("barcode red")||name2.equals("red barcode")||name2.equals("red barcodes")||name2.equals("barcodes red")){//return all the barcodes on the red side
            Point output[]= new Point[6];
            output[0]=barcode[6];
            output[1]=barcode[7];
            output[2]=barcode[8];
            output[3]=barcode[9];
            output[4]=barcode[10];
            output[5]=barcode[11];
            return output;
        }

        if(name2.equals("barcode warehouse")||name2.equals("warehouse barcode")||name2.equals("warehouse barcodes")||name2.equals("barcodes warehouse")){//return all the barcodes on the warehouse side
            Point output[]= new Point[6];
            output[0]=barcode[3];
            output[1]=barcode[4];
            output[2]=barcode[5];
            output[3]=barcode[9];
            output[4]=barcode[10];
            output[5]=barcode[11];
            return output;
        }

        if(name2.equals("barcode carousel")||name2.equals("carousel barcode")||name2.equals("carousel barcodes")||name2.equals("barcodes carousel")){//return all the barcodes on the carousel side
            Point output[]= new Point[6];
            output[0]=barcode[0];
            output[1]=barcode[1];
            output[2]=barcode[2];
            output[3]=barcode[6];
            output[4]=barcode[7];
            output[5]=barcode[8];
            return output;
        }



        for(int i=0;i<barcode.length;i++){//return a single point if the proved name is that of a specific name
            if(barcode[i].name.equals(name)){
                return new Point[]{barcode[i]};
            }
        }
        return null;
    }

    public static Point gethub(String name){//return the hub with the provided name if one exists
        for(int i=0;i<hubs.length;i++){
            if(hubs[i].name.equals(name)){
                return hubs[i];
            }
        }
        return null;
    }

    public static boolean inWareHouse(double x,double y){//return true if the provided point is in a warehouse
        return (x<warehouses[0].x&&y>warehouses[0].y)||(x>warehouses[1].x&&y>warehouses[1].y);
    }

    public static boolean inBlueWarehouse(double x, double y) {//return true if the provided point is in the blue warehouse
        return (x<warehouses[0].x&&y>warehouses[0].y);
    }

    public static boolean inRedWarehouse(double x, double y) {//return true if the provided point is in the red warehouse
        return (x>warehouses[1].x&&y>warehouses[1].y);
    }

}
