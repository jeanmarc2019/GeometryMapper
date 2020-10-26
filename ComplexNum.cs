using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;

public class ComplexNum 
{
    public float r;
    public float i;
    public ComplexNum() {
        r = 0;
        i = 0;
    }
    public ComplexNum(float real, float imag) {
        this.r = real;
        this.i= imag;
    }
    public float mag()
    {
        //sqrt of this
        return (float)Math.Sqrt((this.r*this.r) + (this.i*this.i));
    }
   
    private ComplexNum GCDComplex(ComplexNum a, ComplexNum b)
    {
        //assume a.mag() > b.mag()
        //a/b = x+yi
        //round x and y to closest ints that gives you q
        //r = a-b*q
        //repeat until r=0
        /* ComplexNum r = new ComplexNum(1, 1);
         if (a.mag() > b.mag())
         {

             ComplexNum c = a / b;
             ComplexNum q = new ComplexNum();
             q.r = Math.Round(c.r);
             q.i = Math.Round(c.i);
             r = a - (b * q);
             if (r.r == 0 && r.i == 0)
             {
                 return q;
             }
             else
             {
                 return GCDComplex(r, b);    
             }
         }
         else if()
         {

         }*/
        return null;

    }

    public static ComplexNum operator /(ComplexNum a, ComplexNum b) {
        float real = ((a.r * b.r) + (a.i * b.i)) / (Mathf.Pow(b.r, 2) + Mathf.Pow(b.i, 2));
        float imag = ((a.i * b.r) - (a.r * b.i)) / (Mathf.Pow(b.r, 2) + Mathf.Pow(b.i, 2));
        return new ComplexNum(real, imag);
    }
    public static ComplexNum operator +(ComplexNum a, ComplexNum b) {
        float real = a.r + b.r;
        float imag = a.i + b.i;
        return new ComplexNum(real, imag);
    }
    public static ComplexNum operator -(ComplexNum a, ComplexNum b) {
        float real = a.r - b.r;
        float imag = a.i - b.i;
        return new ComplexNum(real, imag);
    }
    public static ComplexNum operator *(ComplexNum a, ComplexNum b) {

        return new ComplexNum((a.r * b.r) - (a.i * b.i), ((a.r * b.i) + (a.i * b.r)));
    }
    public static ComplexNum operator -(ComplexNum a) {
        return (a - a) - a;
    }
    public override string ToString() {

        string s = "CN:";
        if (this.r > 0)
        {
            s = s + " " + this.r;
            if (this.i > 0) { s = s + "+ " + this.i + "i"; }
            else if (this.i == 0) { s = s + "+ " + this.i + "i"; }
            else if (this.i < 0) { s = s + "- " + (-this.i) + "i"; }
        }
        else if (this.r == 0)
        {
            s = s + " 0";

            if (this.i > 0) { s = s + "+ " + this.i + "i"; }
            else if (this.i == 0) { s = s + "+ " + this.i + "i"; }
            else if (this.i < 0) { s = s + "- " + (-this.i) + "i"; }
        }
        else if (this.r < 0) 
        {
            s = s + " " + this.r;
            if (this.i > 0) { s = s + "+ " + this.i + "i"; }
            else if (this.i == 0) { s = s + "+ " + this.i + "i"; }
            else if (this.i < 0) { s = s + "- " + (-this.i) + "i"; }
        }        
        return s;
    }
}
