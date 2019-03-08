import adsk.core, adsk.fusion, traceback
import math, time
from collections import namedtuple

COMMAND_ID = "trochoid_decelerator"
#COMMAND_ID name
  #Tab
ID_NECESSARY_TAB = "necessary_tab_"
ID_OPTIONAL_TAB  = "optional_tab_"
  #Mode

  #Group
ID_OPT_CH_GROUP    = ID_OPTIONAL_TAB + "centor_hole_group_"
ID_OPT_TGTOD_GROUP = ID_OPTIONAL_TAB + "trochoidal_gear_to_output_disk_group_"

  #item
    #all tab
ID_TV = "realtime_view"
#     #parameter tab
# ID_P_ET = ID_OPTIONAL_TAB + "error_text"
    #nessary tab
ID_NES_IMG   = ID_NECESSARY_TAB + "description_image"
ID_NES_RR    = ID_NECESSARY_TAB + "reducation_ratio_"
ID_NES_EA    = ID_NECESSARY_TAB + "eccentric_amount_"
ID_NES_RGPD  = ID_NECESSARY_TAB + "ring_gear_pin_diameter_"
ID_NES_RGPPD = ID_NECESSARY_TAB + "ring_gear_pin_pitch_diameter_"
ID_NES_CGPN  = ID_NECESSARY_TAB + "trochoidal_gear_plot_num_"
    #optional tab
ID_OPT_IMG    = ID_OPTIONAL_TAB + "description_image"
ID_OPT_CGH_DR = ID_OPT_CH_GROUP + "draw_centor_hole"
ID_OPT_CGH_D  = ID_OPT_CH_GROUP + "centor_hole_diameter"
ID_OPT_DR_CAH  = ID_OPT_TGTOD_GROUP + "draw_trochoid_around_hole"
ID_OPT_DR_DP   = ID_OPT_TGTOD_GROUP + "draw_disk_pin"
ID_OPT_CHOTGOD = ID_OPT_TGTOD_GROUP + "choise_trochoidal_gear_or_output_disk"
ID_OPT_CHOTGOD_AN  = ID_OPT_TGTOD_GROUP + "around_hole_num"
ID_OPT_CHOTGOD_AD  = ID_OPT_TGTOD_GROUP + "around_hole_diameter"
ID_OPT_CHOTGOD_APD = ID_OPT_TGTOD_GROUP + "around_hole_position_diameter"
ID_OPT_CHOTGOD_ON  = ID_OPT_TGTOD_GROUP + "output_disk_num"
ID_OPT_CHOTGOD_OD  = ID_OPT_TGTOD_GROUP + "output_disk_diameter"
ID_OPT_CHOTGOD_OPD = ID_OPT_TGTOD_GROUP + "output_disk_position_diameter"



# コマンドの識別データの設定
COMMAND_NAME = 'create cyclo reducer'
COMMAND_DESCRIPTION = 'サイクロ減速機用曲線作成スクリプト'


##
# @brief 合成シンプソン法による近似積分
# @param func f(x)の関数、例)lambda x:x**3
# @param upper 積分範囲の上限
# @param lower 積分範囲の下限
# @param splitNum 何分割するか、偶数のみ
def compositeSimpson(func,upper,lower, splitNum):
    splitNum=int(splitNum)
    if splitNum&0b1:    #奇数
        splitNum += 1
    h = (upper-lower)/splitNum
    ysum = func(lower) + 4*func(lower+h) + func(upper)
    for i in range(2,splitNum)[::2]:
        ysum += 2*func(lower+i*h) + 4*func(lower+(i+1)*h)
    return h/3*(ysum)

##
# @brief 二分法
def bisectionMethod(func, upper, lower, maxError, maxCalcTimes=100):
    maxError=abs(maxError)
    calcTimes=0
    while True:
        calcTimes+=1
        x = (upper+lower)/2.0
        if (0.0 < func(x)*func(upper)):#符号判定
            upper=x
        else:
            lower=x
        if (upper-lower<=maxError):
            return x
        elif (calcTimes==maxCalcTimes):
            # ui.messageBox("error")
            return x

##
# @brief サイクロ減速機の部品の値を取得するクラス
class CycloidalReducer():
    ##
    # @brief パラメータの設定
    # @param ringPinNum 外ピンの数
    # @param riingPinRadius 外ピン半径
    # @param ringPinPitchRadius 外ピンの配置半径
    # @param eccentricAmount 偏心量
    def __init__(self, ringPinNum, ringPinRadius, ringPinPitchRadius, eccentricAmount):
        self.ringPinNum              = ringPinNum           #外ピン数
        self.ringPinRadius           = ringPinRadius        #外歯半径
        self.ringPinPitchRadius      = ringPinPitchRadius   #外ピン配置半径
        self.trochoidalGearThoothNum = ringPinNum-1         #内歯数
        self.eccentricAmount         = eccentricAmount      #偏心量
        self.reducationRatio = self.trochoidalGearThoothNum / (self.ringPinNum - self.trochoidalGearThoothNum)#減速比

        self.rm = self.ringPinPitchRadius/(self.reducationRatio+1)  #定円半径
        self.rc = self.rm*self.reducationRatio                      #動円半径
        self.rd = self.eccentricAmount                              #動円の描画半径
        self.d  = self.ringPinRadius                                #オフセット量

    ## x of trochoid curve
    # @brief \f$f_{xa}(p)\f$
    def fxa(self, p):
        return (self.rc+self.rm)*math.cos(p) - self.rd*math.cos((self.rc+self.rm)/self.rm*p)
    ## y of trochoid curve
    # @brief \f$f_{ya}(p)\f$
    def fya(self, p):
        return (self.rc+self.rm)*math.sin(p) - self.rd*math.sin((self.rc+self.rm)/self.rm*p)
    ## x of differential trochoid curve
    # @brief \f$ \frac{df_{xa}(p)}{dp} \f$
    def dfxa(self, p):
        return -(self.rc+self.rm)*math.sin(p) + ((self.rc+self.rm)/self.rm)*self.rd*math.sin((self.rc+self.rm)/self.rm*p)
    ## y of differential trochoid curve
    # @brief \f$ \frac{df_{ya}(p)}{dp} \f$
    def dfya(self, p):
        return (self.rc+self.rm)*math.cos(p) - ((self.rc+self.rm)/self.rm)*self.rd*math.cos((self.rc+self.rm)/self.rm*p)
    ## x of 2differential trochoid curve
    def ddfxa(self, p):
        return -(self.rc+self.rm)*math.cos(p) + ((self.rc+self.rm)/self.rm)**2 * self.rd*math.cos((self.rc+self.rm)/self.rm*p)
    ## y of 2differential trochoid curve
    def ddfya(self, p):
        return -(self.rc+self.rm)*math.sin(p) + ((self.rc+self.rm)/self.rm)**2 * self.rd*math.sin((self.rc+self.rm)/self.rm*p)
    ## x of trochoidal parallel curve
    # @brief \f$ f_{xp}(p) \f$
    def fxp(self, p):
        dxa = self.dfxa(p)
        dya = self.dfya(p)
        return self.fxa(p) + self.d*-dya / math.sqrt(dxa**2+dya**2)
    ## y of trochoidal parallel curve
    # @brief \f$ f_{yp}(p) \f$
    def fyp(self, p):
        dxa = self.dfxa(p)
        dya = self.dfya(p)
        return self.fya(p) + self.d*dxa / math.sqrt(dxa**2+dya**2)
    ## x of differential trochoidal parallel curve
    # @brief \f$ \frac{df_{xp}(p)}{dp} \f$
    def dfxp(self, p):
        dxa = self.dfxa(p)
        dya = self.dfya(p)
        ddxa = self.ddfxa(p)
        ddya = self.ddfya(p)
        D=self.d
        return dxa * (1 + D*(-dxa*ddya + dya*ddxa)/(dxa**2+dya**2)**(3/2) )

    ## y of differential trochoidal parallel curve
    # @brief \f$ \frac{df_{yp}(p)}{dp} \f$
    def dfyp(self, p):
        dxa = self.dfxa(p)
        dya = self.dfya(p)
        ddxa = self.ddfxa(p)
        ddya = self.ddfya(p)
        D=self.d
        return dya * (1 + D*(dya*ddxa - dxa*ddya)/(dxa**2+dya**2)**(3/2) )

    ## 周長を求める
    def getPerimeter(self, upper, lower, splitNum=1000):
        dfl = lambda p: math.sqrt(self.dfxp(p)**2 + self.dfyp(p)**2) #周長の微分
        return compositeSimpson(dfl, upper, lower, splitNum)
    ## とある点から周長を一定の距離分なぞった点を取得
    def getConstDistancePoint(self, currentP, distance, upperP):
        f=lambda p: self.getPerimeter(p, currentP, 100)-distance
        return bisectionMethod(f, upperP, currentP, self.pointError)

    ## トロコイド曲線の点をプロット
    # @return (list of [x,y], centor)
    def getTrochoidPoints(self, pointNum, shift=False):
        centor = [self.eccentricAmount,0] if shift else [0,0]
        lastPOneThooth = 2*math.pi/self.trochoidalGearThoothNum
        pOneThooth = [i*lastPOneThooth/pointNum for i in range(pointNum)]

        #残りの歯の点をコピー&回転で作る
        pAllThooth = []
        for i in range(self.trochoidalGearThoothNum):
            pAllThooth += [i*lastPOneThooth+p for p in pOneThooth]

        points=[]
        for p in pAllThooth:
            points.append([self.fxa(p)+centor[0], self.fya(p)+centor[1]])
        return (points, centor)

    ## トロコイド並行曲線の点のプロット
    # @return (list of [x,y], centor)
    def getTrochoidParallelCurvePoints(self, pointNum, shift=True):
        centor = [self.eccentricAmount,0] if shift else [0,0]
        lastPOneThooth = 2*math.pi/self.trochoidalGearThoothNum

        perimeterOneThooth = self.getPerimeter(lastPOneThooth, 0, 1000)/pointNum
        self.pointError = perimeterOneThooth/pointNum/1000000

        pOneThooth=[0]
        for i in range(pointNum)[1:]:
            px = pOneThooth[-1]
            pOneThooth.append( self.getConstDistancePoint(px, perimeterOneThooth, lastPOneThooth) )

        #残りの歯の点をコピー&回転で作る
        pAllThooth = []
        for i in range(self.trochoidalGearThoothNum):
            pAllThooth += [i*lastPOneThooth+p for p in pOneThooth]

        points=[]
        for p in pAllThooth:
            points.append([self.fxp(p)+centor[0], self.fyp(p)+centor[1]])
        return (points, centor)

    # @return [list of ringPins centorXY, radius]
    def getOutpinPoints(self):
        points = []
        for i in range(self.ringPinNum):
            theta = 2*math.pi * (i/self.ringPinNum)
            x = self.ringPinPitchRadius*math.cos(theta)
            y = self.ringPinPitchRadius*math.sin(theta)
            points.append([x,y])
        return (points, self.ringPinRadius)


class DrawCycloReducer():
    def __init__(self, inputs):
        drawingParam = inputsToParameter(inputs)

        app = adsk.core.Application.get()
        design = app.activeProduct

        #アクティブなコンポーネント
        activeComp = design.activeOccurrence.component if design.activeOccurrence else  design.rootComponent

        #部品の親となるコンポーネント
        occTrochoidalGear = activeComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        compReducer = occTrochoidalGear.component
        compReducer.name = "Cycloidal reducer"

        self.cycoroidDecelerator = CycloidalReducer(int(drawingParam.ringPinNum), drawingParam.ringPinDia/2.0,
                                                    drawingParam.ringPinPitchDia/2.0,  drawingParam.eccentricAmount)

        #sketch
        trochoidSketch = compReducer.sketches.add(compReducer.xYConstructionPlane)
        trochoidSketch.name = "Trochoidal gear"+"(rr:"+str(drawingParam.ringPinNum-1)+" ea:"+str(drawingParam.eccentricAmount)+")"
        ringPinSketch = compReducer.sketches.add(compReducer.xYConstructionPlane)
        ringPinSketch.name = "Ring pins"+"(rpd:"+str(drawingParam.ringPinDia)+" rppd:"+str(drawingParam.ringPinDia)+")"
        if drawingParam.isDrawOutputDiskPin:
            outputDiskSketch = compReducer.sketches.add(compReducer.xYConstructionPlane)
            outputDiskSketch.name = "Output disk"

        #スケッチの中身を作成
        self.createTrochoidalGear(trochoidSketch, drawingParam)
        self.createRingGear(ringPinSketch, drawingParam)
        if drawingParam.isDrawCentorHole:
            self.createTrochoidalGearCentorHole(trochoidSketch, drawingParam)
        if drawingParam.isDrawAroundHole:
            self.createTrochoidalGearAroundHole(trochoidSketch, drawingParam)
        if drawingParam.isDrawOutputDiskPin:
            self.createOutputDisk(outputDiskSketch, drawingParam)

    def createTrochoidalGear(self, sketch, drawingParam):
        sketchOriginPoint = sketch.originPoint
        z=0
        #トロコイド曲線の計算
        (trochoidParallelPoints, trochoidalGearCentor) = self.cycoroidDecelerator.getTrochoidParallelCurvePoints(drawingParam.plotDotNum)

        #トロコイド曲線の中心点の描画
        self.trochoidCentorPoint2D = sketch.sketchPoints.add( adsk.core.Point3D.create(trochoidalGearCentor[0], trochoidalGearCentor[1], z) )
        self.distanceDimentionEasy(sketch, self.trochoidCentorPoint2D, sketchOriginPoint)#位置の拘束

        #トロコイド平行曲線の描画
        splinePoints = adsk.core.ObjectCollection.create()
        for (x,y) in trochoidParallelPoints:
            splinePoints.add(adsk.core.Point3D.create(x, y, z))
        trochoidCurve = sketch.sketchCurves.sketchFittedSplines.add(splinePoints)
        trochoidCurve.isClosed = True
        trochoidCurve.isFixed  = True

        #トロコイド曲線の描画
        if False:
            (trochoidPoints, trochoidalGearCentor) = self.cycoroidDecelerator.getTrochoidPoints(drawingParam.plotDotNum, True)
            splinePoints2 = adsk.core.ObjectCollection.create()
            for (xa,ya) in trochoidPoints:
                splinePoints2.add(adsk.core.Point3D.create(xa, ya, z))
            trochoidCurve2 = sketch.sketchCurves.sketchFittedSplines.add(splinePoints2)
            trochoidCurve2.isClosed = True
            trochoidCurve2.isFixed  = True
            trochoidCurve2.isConstruction = True

    def createTrochoidalGearAroundHole(self, sketch, drawingParam):
        sketchOriginPoint = sketch.originPoint
        n = drawingParam.troGearAroundHoleNum
        r = drawingParam.troGearAroundHoleDia/2.0
        pd = drawingParam.troGearAroundHolePosDia
        pdn = drawingParam.plotDotNum
        z=0

        (trochoidalGearPoints, trochoidalGearCentor) = self.cycoroidDecelerator.getTrochoidPoints(pdn, True)

        trochoidCentorPoint2D = sketch.sketchPoints.add( adsk.core.Point3D.create(trochoidalGearCentor[0], trochoidalGearCentor[1], z) )
        self.distanceDimentionEasy(sketch, trochoidCentorPoint2D, sketchOriginPoint)
        fx = lambda theta : pd/2.0 * math.cos(theta) + trochoidalGearCentor[0]
        fy = lambda theta : pd/2.0 * math.sin(theta) + trochoidalGearCentor[1]
        z=0
        firstCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(fx(0), fy(0), z), r)
        self.distanceDimentionEasy(sketch, trochoidCentorPoint2D, firstCircle.centerSketchPoint)#中心位置の拘束
        self.diameterDimentionEasy(sketch, firstCircle)#直径の拘束
        firstLine = sketch.sketchCurves.sketchLines.addByTwoPoints(trochoidCentorPoint2D, firstCircle.centerSketchPoint)
        firstLine.isConstruction=True

        beforeLine = firstLine
        beforeAngleDim = None
        for i in range(n)[1:]:
            theta = (float(i)/n)*2*math.pi
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(fx(theta), fy(theta), z), r)
            sketch.geometricConstraints.addEqual(firstCircle, circle) #直径をイコール拘束
            line = sketch.sketchCurves.sketchLines.addByTwoPoints(trochoidCentorPoint2D, circle.centerSketchPoint)
            line.isConstruction=True #構造線化
            sketch.geometricConstraints.addEqual(firstLine, line) #長さをイコール高速

            angleDim = self.angleDimentionEasy(sketch, beforeLine, line)#角度を拘束
            if beforeAngleDim:
                angleDim.parameter.expression = beforeAngleDim.parameter.name#参照寸法で連動させる

            beforeLine = line
            beforeAngleDim = angleDim

    def createTrochoidalGearCentorHole(self, sketch, drawingParam):
        sketchOriginPoint = sketch.originPoint
        hd = drawingParam.troGearCentorHoleDia
        pdn = drawingParam.plotDotNum
        z=0

        (trochoidalGearPoints, trochoidalGearCentor) = self.cycoroidDecelerator.getTrochoidPoints(pdn, True)

        #円
        p = adsk.core.Point3D.create(trochoidalGearCentor[0], trochoidalGearCentor[1], z)
        circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(p, hd/2.0)
        self.distanceDimentionEasy(sketch, sketchOriginPoint, circle.centerSketchPoint)#中心位置の拘束
        return self.diameterDimentionEasy(sketch, circle)#直径の拘束

    def createRingGear(self, sketch, drawingParam):
        sketchOriginPoint = sketch.originPoint
        comp = sketch.parentComponent
        z=0
        (points, radius) = self.cycoroidDecelerator.getOutpinPoints()

        #一歯目のRingGearを作成
        firstThoothXY = points[0]
        firstCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(firstThoothXY[0], firstThoothXY[1], z), radius)
        self.distanceDimentionEasy(sketch, sketchOriginPoint, firstCircle.centerSketchPoint)#中心位置の拘束
        self.diameterDimentionEasy(sketch, firstCircle)#直径の拘束
        firstLine = sketch.sketchCurves.sketchLines.addByTwoPoints(sketchOriginPoint, firstCircle.centerSketchPoint)
        firstLine.isConstruction=True

        beforeLine = firstLine
        beforeAngleDim = None
        for p in points[1:]:
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(p[0], p[1], z), radius)
            sketch.geometricConstraints.addEqual(firstCircle, circle) #直径の拘束

            line = sketch.sketchCurves.sketchLines.addByTwoPoints(sketchOriginPoint, circle.centerSketchPoint)
            line.isConstruction=True
            sketch.geometricConstraints.addEqual(firstLine, line)

            angleDim = self.angleDimentionEasy(sketch, beforeLine, line)
            if beforeAngleDim:
                angleDim.parameter.expression = beforeAngleDim.parameter.name#参照寸法で連動させる

            beforeLine = line
            beforeAngleDim = angleDim

    #外ピン配置円の中心点の描画
    def createOutputDisk(self, sketch, drawingParam):
        sketchOriginPoint = sketch.originPoint
        z=0
        #外ピン配置円の中心点の描画
        (points, radius) = self.cycoroidDecelerator.getOutpinPoints()

        # dotPoint = adsk.core.Point3D.create(0, 0, z)
        # sketch.sketchPoints.add(dotPoint)

        #外ピンの円を描画
        n              = drawingParam.outDiskPinNum
        positionRadius = drawingParam.outDiskPinPosDia/2.0
        holeRadius     = drawingParam.outDiskPinDia/2.0
        fx = lambda theta : positionRadius * math.cos(theta)
        fy = lambda theta : positionRadius * math.sin(theta)

        firstCircle = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(fx(0), fy(0), z), holeRadius)
        self.distanceDimentionEasy(sketch, sketchOriginPoint, firstCircle.centerSketchPoint)#中心位置の拘束
        self.diameterDimentionEasy(sketch, firstCircle)#直径の拘束
        firstLine = sketch.sketchCurves.sketchLines.addByTwoPoints(sketchOriginPoint, firstCircle.centerSketchPoint)
        firstLine.isConstruction=True

        beforeLine = firstLine
        beforeAngleDim = None
        for i in range(n)[1:]:
            theta = i/n*2*math.pi
            circle = sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(fx(theta), fy(theta), z), holeRadius)
            sketch.geometricConstraints.addEqual(firstCircle, circle) #直径の拘束
            line = sketch.sketchCurves.sketchLines.addByTwoPoints(sketchOriginPoint, circle.centerSketchPoint)
            line.isConstruction=True
            sketch.geometricConstraints.addEqual(firstLine, line)

            angleDim = self.angleDimentionEasy(sketch, beforeLine, line)
            if beforeAngleDim:
                angleDim.parameter.expression = beforeAngleDim.parameter.name#参照寸法で連動させる

            beforeLine = line
            beforeAngleDim = angleDim

    def distanceDimentionEasy(self, sketch, sketchPoint1, sketchPoint2):
        z=0
        textPoint3D = adsk.core.Point3D.create((sketchPoint1.geometry.x+sketchPoint2.geometry.x)/2,
                                               (sketchPoint1.geometry.y+sketchPoint2.geometry.y)/2, z)
        sketch.sketchDimensions.addDistanceDimension(sketchPoint1, sketchPoint2,
                                                     adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation,
                                                     textPoint3D)
        sketch.sketchDimensions.addDistanceDimension(sketchPoint1,sketchPoint2,
                                                     adsk.fusion.DimensionOrientations.VerticalDimensionOrientation,
                                                     textPoint3D)
    def diameterDimentionEasy(self, sketch, circle):
        z=0
        r = circle.radius
        textPoint3D = adsk.core.Point3D.create(circle.centerSketchPoint.geometry.x+r/2,
                                               circle.centerSketchPoint.geometry.y+r/2, z)
        sketch.sketchDimensions.addDiameterDimension(circle, textPoint3D)
    def angleDimentionEasy(self, sketch, line1, line2):
        z=0
        textPoint3D = adsk.core.Point3D.create((line1.geometry.startPoint.x+line1.geometry.endPoint.x+line2.geometry.startPoint.x+line2.geometry.endPoint.x)/3/2,
                                               (line1.geometry.startPoint.y+line1.geometry.endPoint.y+line2.geometry.startPoint.y+line2.geometry.endPoint.y)/3/2,
                                               (line1.geometry.startPoint.z+line1.geometry.endPoint.z+line2.geometry.startPoint.z+line2.geometry.endPoint.z)/3/2)
        return sketch.sketchDimensions.addAngularDimension(line1, line2, textPoint3D)



def inputsToParameter(commandInputs):
    drawingParam = namedtuple("DrawingParam",
                            ("ringPinNum", "ringPinDia", "ringPinPitchDia",
                                "eccentricAmount", "plotDotNum",
                                "troGearAroundHoleNum", "troGearAroundHoleDia", "troGearAroundHolePosDia",
                                "troGearCentorHoleDia",
                                "outDiskPinNum", "outDiskPinDia","outDiskPinPosDia"
                                "isDrawTrochoidalGear", "isDrawRingPin","isDrawCentorHole", "isDrawAroundHole","isDrawOutputDiskPin"
                                ))

    unitsMgr = app.activeProduct.unitsManager

    drawingParam.isDrawTrochoidalGear = True
    drawingParam.isDrawRingPin      = True
    drawingParam.isDrawCentorHole    = commandInputs.itemById(ID_OPT_CGH_DR).value
    drawingParam.isDrawAroundHole    = commandInputs.itemById(ID_OPT_DR_CAH).value
    drawingParam.isDrawOutputDiskPin = commandInputs.itemById(ID_OPT_DR_DP).value

    #necessary item取得
    reducationRatioInput   = commandInputs.itemById(ID_NES_RR)
    eccentricAmountInput   = commandInputs.itemById(ID_NES_EA)
    ringPinDiaInput    = commandInputs.itemById(ID_NES_RGPD)
    ringPinPitchDiaInput = commandInputs.itemById(ID_NES_RGPPD)
    plotNumInput           = commandInputs.itemById(ID_NES_CGPN)
      #itemから値を取得
    drawingParam.ringPinNum    = int(reducationRatioInput.value)+1
    drawingParam.ringPinDia    = unitsMgr.evaluateExpression(ringPinDiaInput.expression)
    drawingParam.ringPinPitchDia = unitsMgr.evaluateExpression(ringPinPitchDiaInput.expression)
    drawingParam.eccentricAmount   = unitsMgr.evaluateExpression(eccentricAmountInput.expression)
    drawingParam.plotDotNum        = int(plotNumInput.value)

    if drawingParam.isDrawCentorHole:
        troGearCentorHoleDiaInput    = commandInputs.itemById(ID_OPT_CGH_D)
        drawingParam.troGearCentorHoleDia = unitsMgr.evaluateExpression(troGearCentorHoleDiaInput.expression)

    if drawingParam.isDrawAroundHole:
        troGearAroundHoleNumInput    = commandInputs.itemById(ID_OPT_CHOTGOD_AN)
        troGearAroundHoleDiaInput    = commandInputs.itemById(ID_OPT_CHOTGOD_AD)
        troGearAroundHolePosDiaInput = commandInputs.itemById(ID_OPT_CHOTGOD_APD)
          #option trochoidal gear
        drawingParam.troGearAroundHoleNum    = int(troGearAroundHoleNumInput.value)
        drawingParam.troGearAroundHoleDia    = unitsMgr.evaluateExpression(troGearAroundHoleDiaInput.expression)
        drawingParam.troGearAroundHolePosDia = unitsMgr.evaluateExpression(troGearAroundHolePosDiaInput.expression)

    if drawingParam.isDrawOutputDiskPin:
        outDiskPinNumInput    = commandInputs.itemById(ID_OPT_CHOTGOD_ON)
        outDiskPinDiaInput    = commandInputs.itemById(ID_OPT_CHOTGOD_OD)
        outDiskPinPosDiaInput = commandInputs.itemById(ID_OPT_CHOTGOD_OPD)
            #option outputDisk
        drawingParam.outDiskPinNum    = int(outDiskPinNumInput.value)
        drawingParam.outDiskPinDia    = unitsMgr.evaluateExpression(outDiskPinDiaInput.expression)
        drawingParam.outDiskPinPosDia = unitsMgr.evaluateExpression(outDiskPinPosDiaInput.expression)
    return drawingParam

def settingComandInputsItem(inputs):
    #item
      #all tab
    testViewInputs = inputs.addBoolValueInput(ID_TV, "Test view", False, "", False)
    testViewInputs.isFullWidth = True
      #necessary tab
    necessaryTabInput = inputs.addTabCommandInput(ID_NECESSARY_TAB, "Necessary param")
    necessaryTabChildInputs = necessaryTabInput.children
        #necessary tab item
    necImageInputs = necessaryTabChildInputs.addImageCommandInput(ID_NES_IMG, "", "image/cyclo_nec.png")
    necImageInputs.isFullWidth = True
    reducationRatioInput = necessaryTabChildInputs.addIntegerSpinnerCommandInput(ID_NES_RR, 'Raducation ratio', 2, 99999, 1, 10)
    reducationRatioInput.tooltip = "ReducationRatio = RingPinNum-1 = cycloidalGear's thooth num"
    necessaryTabChildInputs.addValueInput(ID_NES_EA,   "Eccentric amount",        "mm", adsk.core.ValueInput.createByReal(0.2))
    necessaryTabChildInputs.addValueInput(ID_NES_RGPD, 'Ring pin diameter',       'mm', adsk.core.ValueInput.createByReal(1.0))
    necessaryTabChildInputs.addValueInput(ID_NES_RGPPD,'Ring pin pitch diameter', 'mm', adsk.core.ValueInput.createByReal(8.0))
    necessaryTabChildInputs.addIntegerSpinnerCommandInput(ID_NES_CGPN, "Cycloidal curve plot num par thooth", 2, 99999, 1, 6)
      #optionary tab
    optionTabInput = inputs.addTabCommandInput(ID_OPTIONAL_TAB, "Optionary param")
    optionTabChildInputs = optionTabInput.children
        #optionary item
    optImageInputs = optionTabChildInputs.addImageCommandInput(ID_OPT_IMG, "", "image/cyclo_opt.png")
    optImageInputs.isFullWidth = True
          #centor hole group
    centorHoleGroup = optionTabChildInputs.addGroupCommandInput(ID_OPT_CH_GROUP, "Cycloidal gear centor hole")
    centorHoleInputs = centorHoleGroup.children
    centorHoleInputs.addBoolValueInput(ID_OPT_CGH_DR, "Draw centor hole", True, "", False)
    centorHoleInputs.addValueInput(ID_OPT_CGH_D, "Diameter", "mm", adsk.core.ValueInput.createByReal(1.6))
          #trochoid hole to output disk pin group
    trochoidToOutputGroup = optionTabChildInputs.addGroupCommandInput(ID_OPT_TGTOD_GROUP, "Cycloidal gear to output disk")
    trochoidToOutputInputs = trochoidToOutputGroup.children
    trochoidToOutputInputs.addBoolValueInput(ID_OPT_DR_CAH, "Draw around hole", True, "", False)
    trochoidToOutputInputs.addBoolValueInput(ID_OPT_DR_DP, "Draw output disk pin", True, "", False)
            #trochoid hole to output disk pin item
    holeOrPinSelectInputs = trochoidToOutputInputs.addDropDownCommandInput(ID_OPT_CHOTGOD, "Set about", adsk.core.DropDownStyles.LabeledIconDropDownStyle)
    holeOrPinSelectListItems = holeOrPinSelectInputs.listItems
    holeOrPinSelectListItems.add("Cycloidal gear hole", True)
    holeOrPinSelectListItems.add("Output disk pin", False)
    trochoidToOutputInputs.addIntegerSpinnerCommandInput(ID_OPT_CHOTGOD_AN, "Hole num", 2, 99999, 1, 8)
    trochoidToOutputInputs.addValueInput(ID_OPT_CHOTGOD_AD,  "Hole diameter",           "mm", adsk.core.ValueInput.createByReal(1.2))
    trochoidToOutputInputs.addValueInput(ID_OPT_CHOTGOD_APD, "Centor to hole distance", "mm", adsk.core.ValueInput.createByReal(4.2))
    trochoidToOutputInputs.addIntegerSpinnerCommandInput(ID_OPT_CHOTGOD_ON, "Pin num", 2, 99999, 1, 8)
    trochoidToOutputInputs.addValueInput(ID_OPT_CHOTGOD_OD,  "Pin diameter",            "mm", adsk.core.ValueInput.createByReal(0.8))
    trochoidToOutputInputs.addValueInput(ID_OPT_CHOTGOD_OPD, "Centor to pin distance",  "mm", adsk.core.ValueInput.createByReal(4.2))
        #both mode item
    # optionTabChildInputs.addTextBoxCommandInput(ID_P_ET, "error text", "", 3, True)
      #draw tab


# コマンド間の参照を維持するための、イベントハンドラのグローバル設定
handlers = []


# 入力ダイアログ・クラス
class MyCommandCreatedHandler(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            app = adsk.core.Application.get()
            ui = app.userInterface

            cmd = args.command

            cmd.setDialogInitialSize(300,500)

            # 値を取り出して使用するイベント
            onExecute = MyCommandExecuteHandler()
            cmd.execute.add(onExecute)
            # 終了イベント（デストラクタ）
            onDestroy = MyCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            # 妥当性検証イベント（バリデーション）
            onValidateInputs = MyCommandValidateInputsHandler()
            cmd.validateInputs.add(onValidateInputs)
            #パラメータ変更時に描画するやつ
            onExecutePreview = MyExecutePreviewHandler()
            cmd.executePreview.add(onExecutePreview)

            # keep the handler referenced beyond this function
            # この関数を越えてハンドラの参照を維持します
            handlers.append(onExecute)
            handlers.append(onDestroy)
            handlers.append(onValidateInputs)
            handlers.append(onExecutePreview)

            # 入力ダイアログの値は、commandInputsコレクションのinputsの中に入ります。
            inputs = cmd.commandInputs
            settingComandInputsItem(inputs)

        except:    #定型エラー処理文
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# 入力ダイアログで入力された値が、妥当であるか検証するクラス
class MyCommandValidateInputsHandler(adsk.core.ValidateInputsEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            args.areInputsValid = True
            app = adsk.core.Application.get()
            cmd = args.firingEvent.sender
            unitsMgr = app.activeProduct.unitsManager
            inputs = cmd.commandInputs
            param  = inputsToParameter(inputs)

            if (param.eccentricAmount <=0) or (param.ringPinPitchDia <=0) \
                    or (param.ringPinDia <=0) or (param.ringPinNum<2) \
                    or (param.eccentricAmount*param.ringPinNum >= (param.ringPinPitchDia/2.0)):
                args.areInputsValid = False

            if param.isDrawCentorHole is True:
                if param.troGearCentorHoleDia <=0:
                    args.areInputsValid = False
            if param.isDrawAroundHole is True:
                if param.troGearAroundHoleNum<=0 or param.troGearAroundHoleDia<=0 or param.troGearAroundHolePosDia<=0:
                    args.areInputsValid = False
            if param.isDrawOutputDiskPin is True:
                if param.outDiskPinNum<=0 or param.outDiskPinDia<=0 or param.outDiskPinPosDia<=0:
                    args.areInputsValid = False

            #centor hole
            centorHoleDiaInput  = inputs.itemById(ID_OPT_CGH_D)
            centorHoleDiaInput.isEnabled = inputs.itemById(ID_OPT_CGH_DR).value

            #trochoidal gear to output disk
            trochoidOrOutputInput = inputs.itemById(ID_OPT_CHOTGOD)
            drawAroundHoleInput   = inputs.itemById(ID_OPT_DR_CAH)
            drawOutputPinInput    = inputs.itemById(ID_OPT_DR_DP)
            AroundHoleNumInput         = inputs.itemById(ID_OPT_CHOTGOD_AN)
            AroundHoleDiameterInput    = inputs.itemById(ID_OPT_CHOTGOD_AD)
            AroundHolePosDiameterInput = inputs.itemById(ID_OPT_CHOTGOD_APD)
            outputNumInput         = inputs.itemById(ID_OPT_CHOTGOD_ON)
            outputDiameterInput    = inputs.itemById(ID_OPT_CHOTGOD_OD)
            outputPosDiameterInput = inputs.itemById(ID_OPT_CHOTGOD_OPD)

            isDrawAroundHole = inputs.itemById(ID_OPT_DR_CAH).value
            isDrawOutputDisk = inputs.itemById(ID_OPT_DR_DP).value

            aroundInputList = [AroundHoleNumInput, AroundHoleDiameterInput, AroundHolePosDiameterInput]
            outputInputList = [outputNumInput, outputDiameterInput, outputPosDiameterInput]

            eccentricAmountInput = inputs.itemById(ID_NES_EA)
            eccentricAmountCm = unitsMgr.evaluateExpression(eccentricAmountInput.expression)

            if (drawAroundHoleInput.value is False) and (drawOutputPinInput.value is False):
                trochoidOrOutputInput.isEnabled = False
                for i in aroundInputList + outputInputList:
                    i.isEnabled = False
            else:
                trochoidOrOutputInput.isEnabled = True
                for i in aroundInputList + outputInputList:
                    i.isEnabled = True

            aroundDiaCm  = unitsMgr.evaluateExpression(AroundHoleDiameterInput.expression)
            outputDiaCm  = unitsMgr.evaluateExpression(outputDiameterInput.expression)
            if trochoidOrOutputInput.selectedItem.index == 0:    #trochoidal gear around hole
                for i in aroundInputList:
                    i.isVisible = True
                for i in outputInputList:
                    i.isVisible = False
                outputNumInput.value              = AroundHoleNumInput.value
                outputDiameterInput.expression    = unitsMgr.formatInternalValue(aroundDiaCm - 2*eccentricAmountCm)
                outputPosDiameterInput.expression = AroundHolePosDiameterInput.expression


            else:#trochoidOrOutputInput.selectedItem.index == 1: #output disk pin
                for i in aroundInputList:
                    i.isVisible = False
                for i in outputInputList:
                    i.isVisible = True
                AroundHoleNumInput.value              = outputNumInput.value
                AroundHoleDiameterInput.expression    = unitsMgr.formatInternalValue(outputDiaCm + 2*eccentricAmountCm)
                AroundHolePosDiameterInput.expression = outputPosDiameterInput.expression

            if isDrawAroundHole:
                if (AroundHoleNumInput.value <= 0) or (aroundDiaCm <= 0) or (unitsMgr.evaluateExpression(AroundHolePosDiameterInput.expression) <= 0):
                    args.areInputsValid = False
            if isDrawOutputDisk:
                if (outputNumInput.value <= 0) or (outputDiaCm <= 0) or (unitsMgr.evaluateExpression(outputPosDiameterInput.expression) <= 0):
                    args.areInputsValid = False


        except:    #定型エラー処理文
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# Event handler for the executePreview event.
class MyExecutePreviewHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            command = args.firingEvent.sender
            inputs = command.commandInputs
            testView = inputs.itemById(ID_TV)
            if testView.value:
                testView.value = False
                DrawCycloReducer(inputs)
        except:    #定型エラー処理文
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# 入力ダイアログで入力された値を取り出すクラス
# 入力ダイアログ・クラスにonValidateInputsイベントを設定して使用する
class MyCommandExecuteHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            app = adsk.core.Application.get()
            design = app.activeProduct

            command = args.firingEvent.sender
            inputs = command.commandInputs
            DrawCycloReducer(inputs)

        except:    #定型エラー処理文
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

# 終了処理
class MyCommandDestroyHandler(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()
    def notify(self, args):
        try:
            # コマンドが実行されると、スクリプトが終了します
            # これは、すべてのイベントハンドラを削除する、すべてのグローバルを解放します
            adsk.terminate()
        except:    #定型エラー処理文
            if ui:
                ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))

def run(context):
    ui = None
    try:
        # アプリケーションを取得
        print("addIn start")
        global app
        app = adsk.core.Application.get()

        # ユーザーインターフェースを取得
        global ui
        ui = app.userInterface

        unitsMgr = app.activeProduct.unitsManager

        # ＊＊＊入力ダイアログの処理＊＊＊
        # コマンド定義を作成します。
        cmdDef = ui.commandDefinitions.itemById(COMMAND_ID)
        if not cmdDef:
            cmdDef = ui.commandDefinitions.addButtonDefinition(COMMAND_ID, COMMAND_NAME, COMMAND_DESCRIPTION)

        # 作成されたイベントのコマンドを追加します

        # 入力ダイアログをイベントに追加
        onCommandCreated = MyCommandCreatedHandler()
        cmdDef.commandCreated.add(onCommandCreated)
        # この関数を越えてハンドラの参照を維持します
        handlers.append(onCommandCreated)

        # コマンドを実行します
        cmdDef.execute()

        # 私たちが、発生するイベントハンドラの待機中のため、スクリプトが返るとき、このモジュールが終了するのを防ぎます。
        adsk.autoTerminate(False)

    except:    #定型エラー処理文
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
