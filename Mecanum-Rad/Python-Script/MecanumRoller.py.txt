import adsk.core
import adsk.fusion
import adsk.cam
import traceback
import math

# Berechne x coordinate
def xCoordinate(t, wheelRadius, rollerRadius):
    xs = (t+0.5*((math.sqrt(wheelRadius**2-t**2)-(wheelRadius-rollerRadius))*-t/(math.sqrt(wheelRadius**2-t**2))))*math.sqrt(2)
    return xs

# calc z coordinate
def zCoordinate(t, wheelRadius, rollerRadius):
    zs = math.sqrt((math.sqrt(wheelRadius**2-t**2)-(wheelRadius-rollerRadius))**2 + 2*(1/2*((math.sqrt(wheelRadius**2-t**2) - (wheelRadius-rollerRadius))*(-t)/(math.sqrt(wheelRadius**2-t**2))))**2)
    return zs    

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        
        # Get active design
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
 
        # Get root component in this design
        rootComp = design.rootComponent

        # Create a new sketch on the xy plane.
        sketches = rootComp.sketches
        xzPlane = rootComp.xZConstructionPlane
        sketch = sketches.add(xzPlane)

        # get user params
        rollerWidth = design.userParameters.itemByName("rollerWidth").value # width of roller
        rollerRadius = design.userParameters.itemByName("rollerRadius").value # radius of roller
        wheelRadius = design.userParameters.itemByName("wheelRadius").value # radius of mecanum wheel
        numPoints = design.userParameters.itemByName("numPoints").value # number of points on curve

        last_point = None
        first_point = None
        line = None
        dS = rollerWidth * 0.5 / (numPoints)

        for i in range(0, int(numPoints) + 1):
            
            t = dS * i
            
            point_x = xCoordinate(t, wheelRadius, rollerRadius)
            point_z = zCoordinate(t, wheelRadius, rollerRadius)
            point = adsk.core.Point3D.create(point_x, point_z, 0)
            sketch.sketchPoints.add(point)                
            app.activeViewport.refresh()

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))#Author-
