import adsk.core, adsk.fusion, traceback
import math
from collections import namedtuple


COMMAND_ID = "trochoid_decelerator"
#COMMAND_ID name
  #Tab
ID_NECESSARY_TAB = "necessary_tab_"
ID_OPTIONAL_TAB  = "optional_tab_"
ID_DRAW_TAB      = "drawing_tab_"
  #Mode

  #Group
ID_OPT_CH_GROUP    = ID_OPTIONAL_TAB + "centor_hole_group_"
ID_OPT_TGTOD_GROUP = ID_OPTIONAL_TAB + "trochoidal_gear_to_output_disk_group_"
ID_DR_SE_GROUP     = ID_DRAW_TAB + "separate_group_"

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
    #drawing tab
ID_DR_SE_S = ID_DR_SE_GROUP + "sketch"
ID_DR_SE_C = ID_DR_SE_GROUP + "component"



# コマンドの識別データの設定
COMMAND_NAME = 'create cyclo reducer'
COMMAND_DESCRIPTION = 'サイクロ減速機用曲線作成スクリプト'



class trochoid():
    def __init__(self):
        #ringPinRadius = eccentricAmount
        self.eccentricAmount = 0
        self.trochoidalGearThoothNum = 0
        self.ringPinNum = 0
        self.ringPinPitchRadius=0

    def setParam(self, ringPinNum, ringPinRadius, ringPinPitchRadius,
                   eccentricAmount, offsetValue=0):
        self.trochoidalGearThoothNum = ringPinNum-1          #内歯数
        self.reducationRatio = self.trochoidalGearThoothNum / (self.ringPinNum - self.trochoidalGearThoothNum)#減速比

        self.eccentricAmount = eccentricAmount                #偏心量
        self.ringPinRadius = ringPinRadius                    #外歯半径
        self.ringPinNum = ringPinNum                          #外ピン数
        self.ringPinPitchRadius = ringPinPitchRadius    #外ピン配置半径

        if ringPinRadius < eccentricAmount:
            return False
        if not isinstance(ringPinNum, int):
            return False

    def getTrochoidParallelCurvePoints(self, pointNum=100, shift=True):
        i = self.trochoidalGearThoothNum / (self.ringPinNum - self.trochoidalGearThoothNum)
        rm = self.ringPinPitchRadius/(i+1)
        rc = rm*i
        rd = self.eccentricAmount
        d  = self.ringPinRadius

        points=[]
        thetaRange = list(range(0, pointNum))#+[0]
        for t in thetaRange:
            theta = 2*math.pi * (t/pointNum)

            yd =  (rc+rm)*(math.cos(theta)) - rd*((rc+rm)/rm)*math.cos((rc+rm)/rm*theta)
            xd = -(rc+rm)*(math.sin(theta)) + rd*((rc+rm)/rm)*math.sin((rc+rm)/rm*theta)
            sqXY = math.sqrt(xd**2+yd**2)
            xa = (rc+rm)*math.cos(theta) - rd*math.cos((rc+rm)/rm*theta)
            ya = (rc+rm)*math.sin(theta) - rd*math.sin((rc+rm)/rm*theta)

            x = xa + d/sqXY * -yd
            y = ya + d/sqXY *  xd

            # x=(rc+rm)*math.cos(theta) - rd*math.cos((rc+rm)/rm*theta)
            # y=(rc+rm)*math.sin(theta) - rd*math.sin((rc+rm)/rm*theta)
            points.append([x,y])

        if shift:
            centor = [self.eccentricAmount,0]
            points = self.shiftTrochoidPoints(points, centor)
        else:
            centor = [0,0]
        return (points, centor)


    # @param rc 定円半径
    # @param rm 動円半径
    # @param rd 動円の描画半径
    # @return list of [x,y]
    def getTrochoidPoints(self, pointNum=100, shift=False):
        i = self.trochoidalGearThoothNum / (self.ringPinNum - self.trochoidalGearThoothNum)
        rm = self.ringPinPitchRadius/(i+1)
        rc = rm*i
        rd = self.eccentricAmount

        #ui.messageBox("rm="+str(rm)+"\ni="+ str(i)+"\nrc="+str(rc)+"\nrd="+str(rd))

        points=[]
        thetaRange = list(range(0, pointNum))#+[0]
        for t in thetaRange:
            theta=(2*math.pi/pointNum*t)
            x=(rc+rm)*math.cos(theta) - rd*math.cos((rc+rm)/rm*theta)
            y=(rc+rm)*math.sin(theta) - rd*math.sin((rc+rm)/rm*theta)
            points.append([x,y])

        if shift:
            centor = [self.eccentricAmount,0]
            points = self.shiftTrochoidPoints(points, centor)
        else:
            centor = [0,0]
        return (points, centor)

    # @return [list of ringPins centorXY, radius]
    def getOutpinPoints(self):
        points = []
        for i in list(range(self.ringPinNum)):
            theta = 2*math.pi* i/self.ringPinNum
            x = self.ringPinPitchRadius*math.cos(theta)
            y = self.ringPinPitchRadius*math.sin(theta)
            points.append([x,y])
        return [points, self.ringPinRadius]

    # @param shiftPoints list of [x,y]
    # @param shiftValue [x,y]
    # @return shifted points
    def shiftTrochoidPoints(self, shiftPoints, shiftValueXY):
        shiftedPoints = []
        for p in shiftPoints:
            shiftedPoints.append( [p[0]+shiftValueXY[0], p[1]+shiftValueXY[1]] )
        return shiftedPoints

class DrawCycloReducer():
    def __init__(self, inputs):
        drawingParam = inputsToParameter(inputs)

        app = adsk.core.Application.get()
        design = app.activeProduct

        #アクティブなコンポーネントの描画
        activeComp = design.activeOccurrence.component if design.activeOccurrence else  design.rootComponent

        rgpn  = drawingParam.ringPinNum
        rgpr  = drawingParam.ringPinDia/2.0
        rgppr = drawingParam.ringPinPitchDia/2.0
        e    = drawingParam.eccentricAmount
        pdn  = drawingParam.plotDotNum

        self.cycoroidDecelerator = trochoid()
        self.cycoroidDecelerator.setParam(int(rgpn), rgpr, rgppr, e)

        if drawingParam.isSeparateComponent:
            occTrochoidalGear = activeComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            compTrochoidalGear = occTrochoidalGear.component
            compTrochoidalGear.name = "cycloidal gear"
            occRingPin = activeComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
            compRingPin = occRingPin.component
            compRingPin.name = "ring pins"
            if drawingParam.isDrawOutputDiskPin:
                occOutputDisk = activeComp.occurrences.addNewComponent(adsk.core.Matrix3D.create())
                compOutputDisk = occOutputDisk.component
                compOutputDisk.name = "output disk"
        else:
            compTrochoidalGear = activeComp
            compRingPin      = activeComp
            compOutputDisk    = activeComp

        if drawingParam.isSeparateSketch:
            trochoidGearSketch = compTrochoidalGear.sketches.add(compTrochoidalGear.xYConstructionPlane)
            trochoidGearSketch.name = "cycloidal gear"
            ringPinSketch = compRingPin.sketches.add(compRingPin.xYConstructionPlane)
            ringPinSketch.name = "ring pins"
            if drawingParam.isDrawOutputDiskPin:
                outputDiskSketch = compOutputDisk.sketches.add(compOutputDisk.xYConstructionPlane)
                outputDiskSketch.name = "output disk"
        else:
            ske = compTrochoidalGear.sketches.add(activeComp.xYConstructionPlane)
            ske.name = "cycloidal drive"
            trochoidGearSketch = ske
            ringPinSketch     = ske
            if drawingParam.isDrawOutputDiskPin:
                outputDiskSketch  = ske


        #スケッチ作成
        self.createTrochoidalGear(trochoidGearSketch, drawingParam)
        self.createRingPin(ringPinSketch, drawingParam)

        if drawingParam.isDrawCentorHole:
            self.createTrochoidalGearCentorHole(trochoidGearSketch, drawingParam)
        if drawingParam.isDrawAroundHole:
            self.createTrochoidalGearAroundHole(trochoidGearSketch, drawingParam)
        if drawingParam.isDrawOutputDiskPin:
            self.createOutputDisk(outputDiskSketch, drawingParam)

    def createTrochoidalGear(self, sketch, drawingParam):
        # app = adsk.core.Application.get()
        # ui = app.userInterface

        z=0
        #トロコイド曲線の計算
        (trochoidalGearPoints, trochoidalGearCentor) = self.cycoroidDecelerator.getTrochoidParallelCurvePoints(drawingParam.plotDotNum)

        #トロコイド曲線の中心点の描画
        dotPoint = adsk.core.Point3D.create(trochoidalGearCentor[0], trochoidalGearCentor[1], z)
        a = sketch.sketchPoints.add(dotPoint)

        #トロコイド曲線の描画
        splinePoints = adsk.core.ObjectCollection.create()
        for p in trochoidalGearPoints:
            splinePoint = adsk.core.Point3D.create(p[0], p[1], z)
            splinePoints.add(splinePoint)
        trochoidCurve = sketch.sketchCurves.sketchFittedSplines.add(splinePoints)
        trochoidCurve.isClosed  = True

    def createTrochoidalGearAroundHole(self, sketch, drawingParam):
        # app = adsk.core.Application.get()
        # ui = app.userInterface

        n = drawingParam.troGearAroundHoleNum
        d = drawingParam.troGearAroundHoleDia
        pd = drawingParam.troGearAroundHolePosDia
        pdn = drawingParam.plotDotNum

        z=0

        (trochoidalGearPoints, trochoidalGearCentor) = self.cycoroidDecelerator.getTrochoidPoints(pdn, True)

        #トロコイド曲線の中心点の描画
        dotPoint = adsk.core.Point3D.create(trochoidalGearCentor[0], trochoidalGearCentor[1], z)
        sketch.sketchPoints.add(dotPoint)

        #円
        for i in list(range(n)):
            theta = (i/n)*2*math.pi
            x = pd/2.0 * math.cos(theta) + trochoidalGearCentor[0]
            y = pd/2.0 * math.sin(theta) + trochoidalGearCentor[1]
            sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(x, y, z), d/2.0)

    def createTrochoidalGearCentorHole(self, sketch, drawingParam):
        # app = adsk.core.Application.get()
        # ui = app.userInterface

        hd = drawingParam.troGearCentorHoleDia
        pdn = drawingParam.plotDotNum
        z=0

        (trochoidalGearPoints, trochoidalGearCentor) = self.cycoroidDecelerator.getTrochoidPoints(pdn, True)

        #トロコイド曲線の中心点の描画
        dotPoint = adsk.core.Point3D.create(trochoidalGearCentor[0], trochoidalGearCentor[1], z)
        sketch.sketchPoints.add(dotPoint)

        #円
        p = adsk.core.Point3D.create(trochoidalGearCentor[0], trochoidalGearCentor[1], z)
        sketch.sketchCurves.sketchCircles.addByCenterRadius(p, hd/2.0)

    def createRingPin(self, sketch, drawingParam):
        # app = adsk.core.Application.get()
        # ui = app.userInterface

        z=0
        #外ピン配置円の中心点の描画
        (points, radius) = self.cycoroidDecelerator.getOutpinPoints()
        dotPoint = adsk.core.Point3D.create(0, 0, z)
        sketch.sketchPoints.add(dotPoint)

        #外ピンの円を描画
        for p in points:
            sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(p[0], p[1], z), radius)

    def createOutputDisk(self, sketch, drawingParam):
        # app = adsk.core.Application.get()
        # ui = app.userInterface
        z=0

        #外ピン配置円の中心点の描画
        (points, radius) = self.cycoroidDecelerator.getOutpinPoints()

        dotPoint = adsk.core.Point3D.create(0, 0, z)
        sketch.sketchPoints.add(dotPoint)

        #外ピンの円を描画
        # n=drawingParam.troGearAroundHoleNum
        # positionRadius = drawingParam.troGearAroundHolePosDia/2.0
        # holeRadius     = drawingParam.troGearAroundHoleDia/2.0 - drawingParam.eccentricAmount
        n              = drawingParam.outDiskPinNum
        positionRadius = drawingParam.outDiskPinPosDia/2.0
        holeRadius     = drawingParam.outDiskPinDia/2.0
        for i in list(range(n)):
            theta = i/n*2*math.pi

            x = positionRadius*math.cos(theta)
            y = positionRadius*math.sin(theta)
            sketch.sketchCurves.sketchCircles.addByCenterRadius(adsk.core.Point3D.create(x, y, z), holeRadius)

def inputsToParameter(commandInputs):
    drawingParam = namedtuple("DrawingParam",
                            ("ringPinNum", "ringPinDia", "ringPinPitchDia",
                                "eccentricAmount", "plotDotNum",

                                "troGearAroundHoleNum", "troGearAroundHoleDia", "troGearAroundHolePosDia",
                                "troGearCentorHoleDia",
                                "outDiskPinNum", "outDiskPinDia","outDiskPinPosDia"

                                "isDrawTrochoidalGear", "isDrawRingPin","isDrawCentorHole", "isDrawAroundHole","isDrawOutputDiskPin"
                                "isSeparateSketch", "isSeparateComponent"
                                ))

    unitsMgr = app.activeProduct.unitsManager
    drawingParam.isSeparateSketch    = commandInputs.itemById(ID_DR_SE_S).value
    drawingParam.isSeparateComponent = commandInputs.itemById(ID_DR_SE_C).value

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
    drawingParam.plotDotNum        = int(plotNumInput.value) * drawingParam.ringPinNum

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
    # pCommand.setDialogMinimumSize(500,1)
    #item
      #all tab
    testViewInputs = inputs.addBoolValueInput(ID_TV, "Test view", False, "", False)
    testViewInputs.isFullWidth = True
    # a.isFullWidth=True
      #necessary tab
    necessaryTabInput = inputs.addTabCommandInput(ID_NECESSARY_TAB, "Necessary param")
    necessaryTabChildInputs = necessaryTabInput.children
        #necessary tab item
    necImageInputs = necessaryTabChildInputs.addImageCommandInput(ID_NES_IMG, "", "image/cyclo_nec.png")
    necImageInputs.isFullWidth = True
    necessaryTabChildInputs.addIntegerSpinnerCommandInput(ID_NES_RR, 'Raducation ratio', 2, 99999, 1, 10)
    necessaryTabChildInputs.addValueInput(ID_NES_EA,   "Eccentric amount",        "mm", adsk.core.ValueInput.createByReal(0.2))
    necessaryTabChildInputs.addValueInput(ID_NES_RGPD, 'Ring pin diameter',       'mm', adsk.core.ValueInput.createByReal(1.0))
    necessaryTabChildInputs.addValueInput(ID_NES_RGPPD,'Ring pin pitch diameter', 'mm', adsk.core.ValueInput.createByReal(8.0))
    necessaryTabChildInputs.addIntegerSpinnerCommandInput(ID_NES_CGPN, "Cycloidal curve plot num par thooth", 2, 99999, 1, 5)
      #optionary tab
    optionTabInput = inputs.addTabCommandInput(ID_OPTIONAL_TAB, "optionary param")
    optionTabChildInputs = optionTabInput.children
        #optionary item
    optImageInputs = optionTabChildInputs.addImageCommandInput(ID_OPT_IMG, "", "image/cyclo_opt.png")
    optImageInputs.isFullWidth = True
          #centor hole group
    centorHoleGroup = optionTabChildInputs.addGroupCommandInput(ID_OPT_CH_GROUP, "Cycloidal gear centor hole")
    centorHoleInputs = centorHoleGroup.children
    centorHoleInputs.addBoolValueInput(ID_OPT_CGH_DR, "Draw cycloidal gear centor hole", True, "", False)
    centorHoleInputs.addValueInput(ID_OPT_CGH_D, "Diameter", "mm", adsk.core.ValueInput.createByReal(1.6))
          #trochoid hole to output disk pin group
    trochoidToOutputGroup = optionTabChildInputs.addGroupCommandInput(ID_OPT_TGTOD_GROUP, "Cycloidal gear to output disk")
    trochoidToOutputInputs = trochoidToOutputGroup.children
    trochoidToOutputInputs.addBoolValueInput(ID_OPT_DR_CAH, "Draw cycloidal gear around hole", True, "", False)
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
    drawTabInput = inputs.addTabCommandInput(ID_DRAW_TAB, "Detailed setting")
    drawTabChildInputs = drawTabInput.children
        #separate group
    drawSeparateGroup      = drawTabChildInputs.addGroupCommandInput(ID_DR_SE_GROUP,   "Separate")
    drawSeparateInputs      = drawSeparateGroup.children
          #separate group item
    drawSeparateInputs.addBoolValueInput(ID_DR_SE_S, "sketch", True, "", True)
    drawSeparateInputs.addBoolValueInput(ID_DR_SE_C, "component", True, "", True)

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

            # r = cmd.setDialogInitialSize(5000,5000)
            # ui.messageBox(str(r))

            # 値を取り出して使用するイベント
            onExecute = MyCommandExecuteHandler()
            cmd.execute.add(onExecute)
            # 終了イベント（デストラクタ）
            onDestroy = MyCommandDestroyHandler()
            cmd.destroy.add(onDestroy)
            # 妥当性検証イベント（バリデーション）
            onValidateInputs = MyCommandValidateInputsHandler()
            cmd.validateInputs.add(onValidateInputs)

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

            cmd.setDialogInitialSize(300,500)
            # cmd.setDialogMinimumSize(500,500)


            if param.eccentricAmount <=0:
                args.areInputsValid = False
            if param.ringPinPitchDia <=0:
                args.areInputsValid = False
            if param.ringPinDia <=0:
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

            sepatateSketchInput = inputs.itemById(ID_DR_SE_S)
            sepatateComponentInput = inputs.itemById(ID_DR_SE_C)




            if sepatateComponentInput.value is True:
                sepatateSketchInput.value = True

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
                # create(inputs)
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
            # create(inputs)
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
        global app
        app = adsk.core.Application.get()

        # ユーザーインターフェースを取得
        global ui
        ui = app.userInterface

        # ui.messageBox("hoge")

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