# Fusion360 script for creating Cycloidal Drive sketch

简体中文版本，下文待翻译

This can create part of cycloidal drive
* Cycloida gear (gray)
    * Trochoidal parallel curve
    * Centor hole
    * Around hole
* Ring pin (yellow)
* Output disk pin (green)

<img src="./image/cyclo_Discription_Image_opt.png" width="300">

# How to use
You clone this in  

Windows – `%appdata%\Autodesk\Autodesk Fusion 360\API\Scripts\`  
Mac – `$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/Scripts\`

## Necessary parameter
You need least 5 parameter

<img src="./image/necessay param image.png" width="300">

#### Raducation ratio
raducation ratio = 1 - (ring pin num / cycloidal gear thooth num)  
This script support only max raducation ratio.  
So, ring pin num = cycloidal gear thooth num + 1

####  Eccentric amount
Difference between centor of ring pin pitch ring and centor of cycloidal gear.  

####  Ring pin diameter
diameter

####  Ring pin pitch diameter
Pitch ring is through to the all centor of ring pin.  

####  Cycloidal curve plot num par thooth
This script reder curve in sprine command.  
So, you need to decide point num.  
If few point num, cycloidal curve be inexactitude.  
If a lot of point num, run slow.  



## Optionary parameter
If you want to draw more, you can draw centor hole, around hole and output disk pin.  

<img src="./image/optionary param image.png" width="300">

#### Draw centor hole (Cycloidal gear centor hole)
If you check this, drawn centor hole and you can set diameter.  

#### Diameter (Cycloidal gear centor hole)
Centor hole diameter

#### Draw around hole (Cycloidal gear to output disk)
If you check this, drawn around hole and you can set parameter.  

#### Draw output disk pin (Cycloidal gear to output disk)
If you check this, drawn output disk pin and you can set parameter.  

#### Set about (Cycloidal gear to output disk)
Around hole parameter linked with output disk parameter.  
You can choise set about around hole's parameter or output disk pin's parameter.

#### Hole(pin) num
Around hole or disk pin num.    

#### Hole(pin) diameter
Around hole or disk pin diameter.  
around hole diameter - output disk pin diameter = 2 *  Eccentric amount

#### Centor to hole(pin) distance
Centor of cycloidal gear to around hole distance.  
Or centor of output disk to output disk pin distance.  

## detail setting
You can set unrelated parameter to cycloidal drive.

<img src="./image/detail setting image.png" width="300">

#### sketch (Separate)
If you check this, create 3 sketch(cycloidal gear, ring pins, output disk).  
Else, create 1 sketch.  

#### component (Separate)
If you check this, create 3 component and create sketch in each component.  
Else, create sketch in active component.  
