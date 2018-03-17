/******************************************************************************/
/*                                                                            */
/* MODULE  : DualPlanningClean.cpp                                            */
/*                                                                            */
/* PURPOSE : Dynamic forcefield follow-through paradigm.                      */
/*                                                                            */
/* DATE    : 01/Oct/2015                                                      */
/*                                                                            */
/* CHANGES                                                                    */
/*                                                                            */
/* V1.0  JNI 01/Oct/2015 - Initial development (based on DynamicLearning.cpp) */
/*                                                                            */
/* V1.1  JNI 13/Oct/2015 - Ongoing development.                               */
/*                                                                            */
/* V1.2  HRS 01/Dec/2015 - Development for follow-through experiments.        */
/*                                                                            */
/* V1.3  JNI 22/Nov/2016 - Cleaning up code with HRS.                         */
/*                                                                            */
/* V1.4  HRS 11/May/2017 - Added options for passive wait trials.             */
/******************************************************************************/

#define MODULE_NAME "DualPlanningClean"

/******************************************************************************/

#include <motor.h>

/******************************************************************************/

int     ConfigFileCount=0;
STRING  ConfigFileList[CONFIG_FILES];
int     ConfigIndex;

STRING  DataName="";
STRING  DataFile="";
STRING  TrialListFile="";

STRING  RobotName="";
int     RobotID=ROBOT_INVALID;
BOOL    RobotFT=FALSE;
double  ForceMax=40.0;
matrix  RobotPosition(3,1);
matrix  RobotVelocity(3,1);
matrix  RobotForces(3,1);
double  RobotSpeed;
PMOVE   RobotPMove;

double  PMoveMovementTime=0.7;         // sec
double  PMoveHoldTime=0.1;             // sec
double  PMoveRampTime=0.1;             // sec
double  PMoveSpringConstant=-30.0;     // N/cm
double  PMovePositionTolerance=0.2;    // cm
double  PMoveVelocityTolerance=5.0;    // cm/sec
matrix  PMoveStartPosition(3,1);
matrix  PMoveEndPosition(3,1);
matrix  PMovePosition(3,1);
matrix  PMoveVelocity(3,1);
matrix  PMoveForces(3,1);
int     PMoveState=0;
double  PMoveStateTime=0;
double  PMoveStateRampValue=0;
matrix  PMoveStatePosition(3,1);

matrix  HandleForces(3,1);
matrix  HandleTorques(3,1);

double  LoopTaskFrequency;
double  LoopTaskPeriod;

#define MOVETYPE_OUTANDBACK  0
#define MOVETYPE_OUTTHENBACK 1
#define MOVETYPE_OUTONLY     2

struct STR_TextItem MovementTypeText[] =
{
    { MOVETYPE_OUTANDBACK  ,"OutAndBack" },
    { MOVETYPE_OUTTHENBACK ,"OutThenBack" },
    { MOVETYPE_OUTONLY     ,"OutOnly" },
    { STR_TEXT_ENDOFTABLE },
};

#define MOVEDIR_OUTBACK 0
#define MOVEDIR_OUT     1
#define MOVEDIR_BACK    2

int     MoveTypeDirection[] = { MOVEDIR_OUTBACK,MOVEDIR_OUT,MOVEDIR_OUT };
int     MoveTypeTrials[] = { 1,2,2 };
STRING  MovementTypeString="";
int     MovementType=MOVETYPE_OUTONLY;
int     MovementDirection=MOVEDIR_OUT;

matrix  TextPosition(3,1);
matrix  ViaPosition(3,1);

double  FinishTolerance=0.50;
double  FinishToleranceTime=0.1;

double  ViaToleranceTime=0.05;
double  ViaTimeOutTime=0.15;
double  MovementReactionTimeOut=0.5;
double  MovementDurationTimeOut=0.8;
double  MovementDurationTooFast=0.2;
double  MovementDurationTooSlow=0.4;
TIMER   MovementDurationTimer("MovementDuration");
TIMER   MovementReactionTimer("MovementReaction");
TIMER   MovementFinishedTimer("MovementFinished");
TIMER   PassingViaTimer("PassingVia");
double  PassingViaTime=0.0;

double  ViaNotMovingSpeed=5.0;
TIMER   ViaNotMovingTimer("ViaNotMoving");
double  ViaNotMovingTime=0.0;

TIMER   MovementFirstTimer("MovementFirstTimer");
double  MovementFirstTime=0.0;
double  MovementFirstTooSlow=0.0;
double  MovementFirstTooFast=0.0;

TIMER   MovementSecondTimer("MovementSecondTimer");
double  MovementSecondTime=0.0;
double  MovementSecondTooSlow=0.0;
double  MovementSecondTooFast=0.0;

double  ErrorWait=0.75;
double  TrialDelay=0.3;
double  TrialDelayMax=0.0;
double  TrialDelayOffset=0.0;
double  TrialDelayLambda=0.0;
double  InterTrialDelay=0.5;
double  FeedbackTime=0.5;
double  NotMovingSpeed=1; // cm/sec
double  NotMovingTime=0.1;
TIMER   NotMovingTimer("NotMoving");

#define RESTBREAK_MAX  30
int     RestBreakCount=0;
int     RestBreakIndex=0;
int     RestBreakTrials[RESTBREAK_MAX+1];
BOOL    RestBreakHere=FALSE;
double  RestBreakSeconds=30.0;
double  RestBreakRemainSeconds=0.0;
double  RestBreakRemainPercent=0.0;
double  RestBreakTrialsPercent=0.0;
double  RestBreakMinutesPerTrial=0.0;
double  RestBreakMinutesRemaining=0.0;

double  TeapotSize=10.0;        // Size of rotating teapot (cm).
double  TeapotRotateSpeed=45.0; // Speed of rotation (deg/sec).

matrix  CursorPosition(3,1);
int     CursorColor=RED;           
STRING  CursorColorText="RED";
double  CursorRadius=0.5;

matrix  ForceFieldForces(3,1);
BOOL    ForceFieldStarted=FALSE;
matrix  ForceFieldPosition(3,1);
double  ForceFieldAngle=0.0;
RAMPER  ForceFieldRamp;
double  ForceFieldRampValue=0.0;
double  ForceFieldRampTime=0.05;

RAMPER  ChannelWidthRamp;
double  ChannelWidthRampTime=0.05;
double  ChannelWidthInitial=0.0;
double  ChannelWidth=0.0;

double  MovedTooFarDistance=0.0;
BOOL    MovedTooFarFlag=FALSE;
double  MissedViaPointDistance=0.0;
BOOL    MissedViaPointFlag=FALSE; 

int     StartColor=WHITE;
int     NotStartColor=GREY;
double  StartRadius=0.5;
double  StartTolerance=0.5;

matrix  TargetPosition(3,1);
int     TargetColor=YELLOW;
STRING  TargetColorText="YELLOW";
int     TargetHitColor=YELLOW;
STRING  TargetHitColorText="YELLOW";
double  TargetRadius=0.5;
double  TargetOutlineWidth=0.1;
int     ViaColor=GREY;
int     ViaPassedColor=GREY;
double  ViaRadius=0.5;
double  ViaHeight=0.0;
double  ViaWidth=0.0;
double  ViaEntryAngle=45.0;
matrix  ViaEntryPosition(3,1);
double  ViaEntryLineLength=0.6;
double  ViaEntryLineWidth=2.0;

#define VIA_CIRCLE    0
#define VIA_RECTANGLE 1
int     ViaType=VIA_CIRCLE;

TIMER_Interval  RobotForcesFunctionLatency("ForcesFunction");
double          ForcesFunctionLatency;
TIMER_Frequency RobotForcesFunctionFrequency("ForcesFunction");
double          ForcesFunctionPeriod;

TIMER_Frequency GraphicsDisplayFrequency("DisplayFrequency");
TIMER_Frequency GraphicsIdleFrequency("IdleFrequency");
TIMER_Interval  GraphicsDisplayLatency("DisplayLatency");
TIMER_Interval  GraphicsSwapBufferLatency("SwapBufferLatency");
TIMER_Interval  GraphicsClearStereoLatency("ClearStereoLatency");
TIMER_Interval  GraphicsClearMonoLatency("ClearMonoLatency");

double  GraphicsVerticalRetraceSyncTime=0.01;   // Time (sec) before vertical retrace to draw graphics frame
double  GraphicsVerticalRetraceCatchTime=0.05;  // Time (msec) to devote to catching vertical retrace
TIMER   GraphicsTargetTimer("GraphicsTarget");

int     GraphicsMode=GRAPHICS_DISPLAY_2D;
int     GraphicsBackGround=LIGHTBLUE;

STRING  GraphicsString="";

// List of WAV files for beeping.
struct WAVELIST WaveList[] = 
{
    { "HighBip","HIGH_BIP.WAV",NULL },
    { "LowBip" ,"LOW_BIP.WAV" ,NULL },
    { "Bip"    ,"MID_BIP.WAV" ,NULL },
    { "","",NULL },
};

TIMER_Interval WaveListPlayInterval("WaveListPlay");

#define FRAMEDATA_ROWS 10000
MATDAT FrameData("FrameData");
BOOL   FrameRecord=FALSE;

TIMER  TrialTimer("Trial");
TIMER  InterTrialDelayTimer("InterTrialDelay");
double TrialTime;
double TrialDuration=0.0;
int    Trial;
BOOL   TrialRunning=FALSE;

// Field types.
#define FIELD_NONE       0
#define FIELD_VISCOUS    1
#define FIELD_CHANNEL    2
#define FIELD_PMOVE      3
#define FIELD_2DSPRING   4
#define FIELD_SAMEASLAST 5
#define FIELD_MAX        6 

// Target context types.
#define TARGET_STATIC_ON	0
#define TARGET_APPEAR		1
#define TARGET_STOP		2
#define TARGET_CENTRAL_ONLY	3
#define TARGET_VISUAL_ONLY	4
#define TARGET_DUAL_OFF	        5
#define DUAL_PLANNING           6
#define TARGET_STATIC_GO	7
#define DUAL_PLANNING_GO        8
#define PASSIVE_WAIT		9
#define PASSIVE_MOVE		10

BOOL PassiveWaitFirstFlag=FALSE;
BOOL PassiveWaitLastFlag=FALSE;

BOOL     ContextFullMovementFlag[] = { TRUE,TRUE,FALSE,FALSE,FALSE,FALSE,TRUE,FALSE,FALSE,FALSE,FALSE };
BOOL     ContextSingleMovementFlag[] = { FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,TRUE,TRUE,TRUE,FALSE };
DATAPROC ContextFullMovementTimeData("ContextFullMovementTimeData");
double   PostMoveDelayTime=0.0;
double   PostMoveDelayInit=0.0;

int     Trials=0;
int     TotalTrials;
int     TrialOffset;
int     TrialPhase=0;
int     TrialPhaseLast=0;
int     PhaseCount=0;
int     PhaseIndex=0;
int     FieldTrials[FIELD_MAX];

#define FIELD_INDEX     128
#define FIELD_CONSTANTS 8

int     FieldIndexType[FIELD_INDEX];
double  FieldIndexConstants[FIELD_INDEX][FIELD_CONSTANTS];
double  FieldIndexAngle[FIELD_INDEX];
int     FieldIndexTrialCount[FIELD_INDEX];
int     FieldIndexContextType[FIELD_INDEX];
double  FieldIndexContextConstants[FIELD_INDEX][FIELD_CONSTANTS];

#define PHASE_MAX    32
int     PhaseTrialRange[PHASE_MAX][2];
int     PhaseFieldIndex[PHASE_MAX][FIELD_INDEX];
int     PhaseFieldIndexCount[PHASE_MAX];
BOOL    PhaseFieldPermute[PHASE_MAX];
PERMUTELIST PhaseFieldIndexPermute;

// Permute list objects to randomize targets.
PERMUTELIST TargetPermute; 

MATDAT TrialData("TrialData");

// Trial data variables.
int    FieldIndex;
int    FieldType;
int    FieldLastType;
double FieldConstants[FIELD_CONSTANTS];
double FieldLastConstants[FIELD_CONSTANTS];
double FieldLastAngle;
double FieldAngle;
int    ContextType;
int    ContextTypeLast;
int    ContextTypeNext;

double ContextConstants[FIELD_CONSTANTS];
// 0 - Target Angle
// 1 - Symmetry Axis Angle
// 2 - Target Resolve Distance
// 3 - Movement Order
// 4 - Not Used
// 5 - Not Used
// 6 - Not Used
// 7 - Not Used

double  SymmetryAxisAngle;
double  TargetResolveDistance;
BOOL    TargetResolveFlag=FALSE;
int     MovementOrderType;
#define ORDER_FOLLOW_THROUGH  0
#define ORDER_LEAD_IN         1
#define ORDER_SINGLE_MOVEMENT 2

int     ChannelOrderType;
#define CHANNEL_FIRST  0
#define CHANNEL_SECOND 1

double TargetAngle;
double MovementReactionTime=0.0;
double MovementDurationTime=0.0;
matrix StartPosition(3,1);
matrix FinishPosition(3,1);

#define MISS_TRIAL_TYPES 10
int     MissTrials=0;
int     MissTrialsType[MISS_TRIAL_TYPES];
int     MissTrialsTotal=0;
int     MissTrialsTypeTotal[MISS_TRIAL_TYPES];
double  MissTrialsPercent=0.0;

#define MISS_TRIAL_TIMEOUT          0
#define MISS_TRIAL_VIAENTRY         1
#define MISS_TRIAL_MISSEDVIA        2
#define MISS_TRIAL_VIATOOLONG       3
#define MISS_TRIAL_VIATOOSHORT      4
#define MISS_TRIAL_ROBOTINACTIVE    5
#define MISS_TRIAL_FRAMEDATAFULL    6
#define MISS_TRIAL_MOVETOOSOON      7

double MovementFirstDistance=10.0;
double MovementSecondDistance=10.0;

int    RobotFieldType=FIELD_NONE;
double RobotFieldConstants[FIELD_CONSTANTS];
double RobotFieldAngle;
matrix RobotFieldMatrix;

TIMER   ExperimentTimer("Experiment");
double  ExperimentTime;
double  ExperimentSeconds;
double  ExperimentMinutes;

/******************************************************************************/

#define STATE_INITIALIZE     0
#define STATE_SETUP          1
#define STATE_HOME           2
#define STATE_START          3
#define STATE_DELAY          4
#define STATE_GO             5
#define STATE_MOVEWAIT       6
#define STATE_MOVING0        7
#define STATE_VIAPOINT       8
#define STATE_MOVING1        9
#define STATE_POSTMOVEDELAY 10
#define STATE_FINISH        11
#define STATE_FEEDBACK	    12
#define STATE_NEXT          13
#define STATE_INTERTRIAL    14
#define STATE_EXIT          15
#define STATE_TIMEOUT       16
#define STATE_ERROR         17
#define STATE_REST          18
#define STATE_MAX           19

int   State=STATE_INITIALIZE;
int   StateLast;
BOOL  StateFirstFlag=FALSE;
int   StateGraphics=STATE_INITIALIZE;
int   StateGraphicsLast;
char *StateText[] = { "Initialize","Setup","Home","Start","Delay","Go","MoveWait","Moving0","ViaPoint","Moving1","PostMoveDelay","Finish","Feedback","Next","InterTrial","Exit","TimeOut","Error","Rest" };
BOOL  StateLoopTask[STATE_MAX] = { FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,TRUE,TRUE,TRUE,TRUE,TRUE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE };
TIMER StateTimer("State");
TIMER StateGraphicsTimer("StateGraphics");
int   StateErrorResume;

/******************************************************************************/

void ProgramExit( void );

/******************************************************************************/

void WaveListPlay( char *name )
{
    // Play WAV file (does interval timing).
    WaveListPlayInterval.Before();
    WAVELIST_Play(WaveList,name);
    WaveListPlayInterval.After();
}

/******************************************************************************/

void ConfigSetup( void )
{
int i;

    // Reset configuration variable list.
    CONFIG_reset();

    // Set up variable list for configuration.
    CONFIG_set(VAR(RobotName));
    CONFIG_setBOOL(VAR(RobotFT));
    CONFIG_set(VAR(ForceMax));
    CONFIG_set("GraphicsSyncTime",GraphicsVerticalRetraceSyncTime);
    CONFIG_set("GraphicsCatchTime",GraphicsVerticalRetraceCatchTime);
    CONFIG_set(VAR(TextPosition));
    CONFIG_set("CursorColor",CursorColorText);
    CONFIG_set(VAR(CursorRadius));
    CONFIG_set(VAR(MovementFirstDistance));
    CONFIG_set(VAR(MovementSecondDistance));
    CONFIG_set("TargetColor",TargetColorText);
    CONFIG_set(VAR(TargetRadius));
    CONFIG_set(VAR(TargetOutlineWidth));
    CONFIG_set(VAR(ViaRadius));
    CONFIG_set(VAR(ViaHeight));
    CONFIG_set(VAR(ViaWidth));
    CONFIG_set(VAR(ViaEntryAngle));
    CONFIG_set(VAR(ViaPosition));
    CONFIG_set(VAR(MovedTooFarDistance));
    CONFIG_set(VAR(MissedViaPointDistance));
    CONFIG_set(VAR(StartRadius));
    CONFIG_set(VAR(StartTolerance));
    CONFIG_set(VAR(FinishTolerance));
    CONFIG_set(VAR(FinishToleranceTime));
    CONFIG_set(VAR(ViaToleranceTime));
    CONFIG_set(VAR(ViaNotMovingSpeed));
    CONFIG_set(VAR(ViaTimeOutTime));
    CONFIG_set(VAR(PostMoveDelayInit));
    CONFIG_set("MovementType",MovementTypeString);
    CONFIG_set(VAR(MovementReactionTimeOut));
    CONFIG_set(VAR(MovementDurationTimeOut));
    CONFIG_set(VAR(MovementDurationTooFast));
    CONFIG_set(VAR(MovementDurationTooSlow)); 
    CONFIG_set(VAR(PMoveMovementTime)); 

    CONFIG_set(VAR(MovementFirstTooFast)); 
    CONFIG_set(VAR(MovementFirstTooSlow)); 
    CONFIG_set(VAR(MovementSecondTooFast)); 
    CONFIG_set(VAR(MovementSecondTooSlow)); 

    CONFIG_set(VAR(ErrorWait));
    CONFIG_set(VAR(TrialDelay));
    CONFIG_set(VAR(TrialDelayMax));
    CONFIG_set(VAR(TrialDelayOffset));
    CONFIG_set(VAR(TrialDelayLambda));
    CONFIG_set(VAR(InterTrialDelay));
    CONFIG_set(VAR(FeedbackTime));
    CONFIG_set(VAR(NotMovingSpeed));
    CONFIG_set(VAR(NotMovingTime));
    CONFIG_set(VAR(ForceFieldRampTime));
    CONFIG_set(VAR(ChannelWidthRampTime));
    CONFIG_set(VAR(ChannelWidthInitial));

    CONFIG_setBOOL(VAR(RestBreakHere));
    CONFIG_set(VAR(RestBreakTrials),RESTBREAK_MAX);
    CONFIG_set(VAR(RestBreakSeconds));

    CONFIG_set(VAR(Trials));	

    for( i=0; (i < FIELD_INDEX); i++ )
    {
        CONFIG_label(STR_stringf("FieldType%d",i),FieldIndexType[i]);
        CONFIG_set("FieldConstants",FieldIndexConstants[i],FIELD_CONSTANTS);
        CONFIG_set("FieldAngle",FieldIndexAngle[i]);
        CONFIG_set("FieldContextType",FieldIndexContextType[i]);
        CONFIG_set("FieldContextConstants",FieldIndexContextConstants[i],FIELD_CONSTANTS);
    }

    for( i=0; (i < PHASE_MAX); i++ )
    {
        CONFIG_label(STR_stringf("PhaseTrials%d",i),PhaseTrialRange[i],2);
        CONFIG_set("FieldIndex",PhaseFieldIndex[i],FIELD_INDEX);
        CONFIG_setBOOL("FieldPermute",PhaseFieldPermute[i]);
    }
}

/******************************************************************************/

void ConfigInit( void )
{
static BOOL first=TRUE;
int i,j;

    if( first )
    {
        first = FALSE;

        for( i=0; (i < FIELD_INDEX); i++ )
        {
            FieldIndexType[i] = FIELD_NONE;
            FieldIndexAngle[i] = 0.0;
            FieldIndexTrialCount[i] = 0;
            FieldIndexContextType[i] = TARGET_STATIC_ON;
 
            for( j=0; (j < FIELD_CONSTANTS); j++ )
            {
                FieldIndexConstants[i][j] = 0.0;
                FieldIndexContextConstants[i][j] = 0.0;
            }
        }

        for( i=0; (i <= RESTBREAK_MAX); i++ )
        {
            RestBreakTrials[i] = 0;
        }

        RestBreakCount = 0;
        RestBreakIndex = 0;
    }

    Trials = 0;
    RestBreakHere = FALSE;

    for( i=0; (i < PHASE_MAX); i++ )
    {
        PhaseTrialRange[i][0] = 0;
        PhaseTrialRange[i][1] = 0;

        PhaseFieldPermute[i] = FALSE;
        PhaseFieldIndexCount[i] = 0;

        for( j=0; (j < FIELD_INDEX); j++ )
        {
            PhaseFieldIndex[i][j] = -1;
        }
    }
}

/******************************************************************************/

BOOL ConfigLoad( char *file )
{
int i;
BOOL ok=TRUE;

    // Setup and initialize configuration variables.
    ConfigSetup();
    ConfigInit();

    // Load configuration file.
    if( !CONFIG_read(file) )
    {
        printf("ConfigLoad(%s) Cannot read file.\n",file);
        return(FALSE);
    }

    if( !GRAPHICS_ColorCode(CursorColor,CursorColorText) )
    {
        printf("ConfigLoad(%s) Invalid color (%s).\n",file,CursorColorText);
        ok = FALSE;
    }

    if( !GRAPHICS_ColorCode(TargetColor,TargetColorText) )
    {
        printf("ConfigLoad(%s) Invalid color (%s).\n",file,TargetColorText);
        ok = FALSE;
    }

    if( STR_null(RobotName) )
    {
        printf("No robot specified.\n");
        ok = FALSE;
    }
    
    // Count the phases
    for( PhaseCount=0,i=0; (i < PHASE_MAX); i++ )
    {
        if( (PhaseTrialRange[i][0] == 0) || (PhaseTrialRange[i][1] == 0) )
        {
            PhaseTrialRange[i][0] = 0;
            PhaseTrialRange[i][1] = 0;

            continue;
        }

        PhaseCount++;

        for( PhaseFieldIndexCount[i]=0; (PhaseFieldIndexCount[i] < FIELD_INDEX); )
        {
            if( PhaseFieldIndex[i][PhaseFieldIndexCount[i]] == -1 )
            {
                break;
            }

            PhaseFieldIndexCount[i]++;

        }

        if( PhaseFieldIndexCount[i] == 0 )
        {
            printf("ConfigLoad(%s) Phase=%d FieldIndex not specified.\n",file,i);
            ok = FALSE;
        }
    }

    if( !STR_null(MovementTypeString) )
    {
        if( (MovementType=STR_TextCode(MovementTypeText,MovementTypeString)) == STR_NOTFOUND )
        {
            printf("ConfigLoad(%s) Invalid movement type (%s).\n",file,MovementTypeString);
            ok = FALSE;
        }
    }

    // Count the rest-breaks in case they have been specified.
    for( RestBreakCount=0; ((RestBreakCount < RESTBREAK_MAX) && (RestBreakTrials[RestBreakCount] != 0)); RestBreakCount++ );

    if( (ViaHeight*ViaWidth) != 0.0 )
    {
        ViaType = VIA_RECTANGLE;
    }

    // Don't print configuration variables because there are too many.
    // printf("ConfigLoad(%s) Load %s.\n",file,STR_OkFailed(ok));
    // CONFIG_list();

    return(ok);
}

/******************************************************************************/

void GraphicsDisplayText( char *string, float size, matrix &pos )
{
static matrix p;
void *font=GLUT_STROKE_MONO_ROMAN;
int i,w;
//float s=size*0.015;
float s=size*0.01;


    p = pos;
    w = strlen(string);

    p(1,1) -= (w / 2) * (s * 100.0);
  
    glPushMatrix();

    glLineWidth(2.0);
    translate(p);
    glScalef(s,s,1);

    GRAPHICS_ColorSet(WHITE);

    for( i=0; (i < w); i++ )
        glutStrokeCharacter(font,string[i]);

    glPopMatrix();
}

/******************************************************************************/

void GraphicsDisplayText( void )
{
static matrix P(3,1);

    if( !STR_null(GraphicsString) )
    {
        P = TextPosition;
	GraphicsDisplayText(GraphicsString,1.0,P);
    }
}

/******************************************************************************/

void GraphicsText( char *text )
{
    if( text != NULL )
    {
        strncpy(GraphicsString,text,STRLEN);
    }
}

/******************************************************************************/

void FrameProcess( void )
{
    // Is frame recording in progres.
    if( !FrameRecord )
    {
        return;
    }

    // Load GRAPHICS-related frame data variables.
    GRAPHICS_FrameData();

    // Save current variables for FrameData.
    FrameData.RowSave();
}

/******************************************************************************/

void FrameStart( void )
{
    // Start recording frame data.
    FrameData.Reset();
    FrameRecord = TRUE;
}

/******************************************************************************/

void FrameStop( void )
{
    // Stop recording frame data.
    FrameRecord = FALSE;
}

/******************************************************************************/

BOOL RestBreakNow( void )
{
BOOL flag=FALSE;

    if( (RestBreakIndex < RestBreakCount) && (Trial == RestBreakTrials[RestBreakIndex]) )
    {
        flag = TRUE;
        RestBreakIndex++;

        RestBreakTrialsPercent = 100.0 * (double)Trial / (double)TotalTrials;
        RestBreakMinutesPerTrial = ExperimentTimer.ElapsedMinutes() / (double)Trial;
        RestBreakMinutesRemaining = RestBreakMinutesPerTrial * (double)(TotalTrials-Trial);
        printf("RestBreak=%d/%d, Time=%.0lf(sec), Trial=%d/%d (%.0lf%% done, %.1f minutes remaining)\n",RestBreakIndex,RestBreakCount,RestBreakSeconds,Trial,TotalTrials,RestBreakTrialsPercent,RestBreakMinutesRemaining);
    }

    return(flag);
}

/******************************************************************************/

void StateProcessLoopTask( void );

/******************************************************************************/

void RobotPMoveUpdate( matrix &F )
{
    PMovePosition(1,1) = RobotPosition(1,1);
    PMovePosition(2,1) = RobotPosition(2,1);
    PMovePosition(3,1) = 0.0;

    PMoveVelocity(1,1) = RobotVelocity(1,1);
    PMoveVelocity(2,1) = RobotVelocity(2,1);
    PMoveVelocity(3,1) = 0.0;

    F.zeros();

    if( RobotPMove.Update(PMovePosition,PMoveVelocity,PMoveForces) )
    {
        F(1,1) = PMoveForces(1,1);
        F(2,1) = PMoveForces(2,1);
        F(3,1) = 0.0;
    }

    RobotPMove.CurrentState(PMoveState,PMoveStateTime,PMoveStateRampValue,PMoveStatePosition);
}

/******************************************************************************/

void RobotPMoveStart( void )
{
BOOL ok;

    PMoveStartPosition(1,1) = RobotPosition(1,1);
    PMoveStartPosition(2,1) = RobotPosition(2,1);
    PMoveStartPosition(3,1) = 0.0;

    PMoveEndPosition(1,1) = FinishPosition(1,1);
    PMoveEndPosition(2,1) = FinishPosition(2,1);
    PMoveEndPosition(3,1) = 0.0;

    ok = RobotPMove.Start(PMoveStartPosition,PMoveEndPosition);
    printf("RobotPMove.Start(...) %s.\n",STR_OkFailed(ok));

   // GraphicsText("Relax Arm");
}

/******************************************************************************/

BOOL RobotPMoveFinished( void )
{
BOOL flag;

    flag = RobotPMove.Finished();
    return(flag);
}

/******************************************************************************/

BOOL RobotPMoveOpen( void )
{
BOOL ok;
matrix SC(3,1),PT(3,1),VT(3,1);

    SC(1,1) = PMoveSpringConstant;
    SC(2,1) = PMoveSpringConstant;
    SC(3,1) = 0.0;

    PT(1,1) = PMovePositionTolerance;
    PT(2,1) = PMovePositionTolerance;
    PT(3,1) = 0.0;

    VT(1,1) = PMoveVelocityTolerance;
    VT(2,1) = PMoveVelocityTolerance;
    VT(3,1) = 0.0;

    ok = RobotPMove.Open(ROBOT_DOFS,PMoveMovementTime,SC,PT,VT,PMoveHoldTime,PMoveRampTime);

    return(ok);
}

/******************************************************************************/

void ForceFieldStart( void )
{
static matrix D(3,1);

    if( ForceFieldStarted )
    {
        return;
    }

    if( (ContextType == PASSIVE_WAIT) && !PassiveWaitFirstFlag )
    {
        return;
    }

    ForceFieldStarted = TRUE;
    ForceFieldAngle = 0.0;

    if( FieldType == FIELD_PMOVE )
    {
        ForceFieldRamp.One();
        RobotPMoveStart();
    }
    else
    {
        if( (RobotFieldType == FIELD_CHANNEL) && (ChannelOrderType == CHANNEL_SECOND) )
        {
            ForceFieldPosition = RobotPosition;
            D = ForceFieldPosition - FinishPosition;
            ForceFieldAngle = R2D(atan2(D(1,1),-D(2,1)));
            //printf("ForceFieldAngle=%.1lf(deg)\n",ForceFieldAngle);
            ChannelWidthRamp.Down();
       }
        else
        {
            ForceFieldPosition = RobotPosition;
        }

        ForceFieldRamp.Up();
    }
}

/******************************************************************************/

void ForceFieldStop( void )
{
    printf("\nForceFieldStop: ContextType=%d,PassiveWaitLastFlag=%d,FieldType=%d,\n",ContextType,PassiveWaitLastFlag,FieldType);

    if( ((ContextType == PASSIVE_WAIT) && !PassiveWaitLastFlag) || (FieldType == FIELD_SAMEASLAST) )
    {
        return;
    }

    printf("ForceFieldStop: DONE!\n\n");

    if( ForceFieldStarted )
    {
        ForceFieldRamp.Down();
        ForceFieldStarted = FALSE;
    }
}

/******************************************************************************/

void RobotForcesFunction( matrix &position, matrix &velocity, matrix &forces )
{
static int i,j;
static matrix P,V,R,_R;
static matrix P1,V1,R1,_R1;
static double d, dx, dy, L;

    // Monitor timing of Forces Function (values saved to FrameData).
    ForcesFunctionPeriod = RobotForcesFunctionFrequency.Loop();
    RobotForcesFunctionLatency.Before();

    TrialTime = TrialTimer.ElapsedSeconds();

    // Kinematic data passed from robot API.
    RobotPosition = position;
    RobotVelocity = velocity;
    RobotSpeed = norm(RobotVelocity);

    // Zero forces.
    ForceFieldForces.zeros();
    RobotForces.zeros();

   // Get Force/Torque sensor if required.
    if( RobotFT && ROBOT_SensorOpened_DAQFT(RobotID) )
    {
        ROBOT_SensorRead(RobotID);
        ROBOT_Sensor_DAQFT(RobotID,HandleForces,HandleTorques);
    }

    P = RobotPosition;
    CursorPosition = RobotPosition;

    // Process force-field type.
    //switch( ForceFieldStarted ? RobotFieldType : FIELD_NONE )
    switch( RobotFieldType )
    {
        case FIELD_NONE :
           break;

        case FIELD_VISCOUS :   // Viscous force field.
           ForceFieldForces = RobotFieldConstants[0] * RobotFieldMatrix * RobotVelocity;
           break;

        case FIELD_CHANNEL :
           // Next rotate position along the channel between the home position and the via point.

           ChannelWidth = ChannelWidthInitial * ChannelWidthRamp.RampCurrent();

           if( MovementOrderType == ORDER_SINGLE_MOVEMENT )
           {
               SPMX_romxZ(D2R(SymmetryAxisAngle+TargetAngle-ForceFieldAngle),R);
               SPMX_romxZ(D2R(-(SymmetryAxisAngle+TargetAngle-ForceFieldAngle)),_R);
           }
           else
           {
               SPMX_romxZ(D2R(SymmetryAxisAngle-ForceFieldAngle),R);
               SPMX_romxZ(D2R(-(SymmetryAxisAngle-ForceFieldAngle)),_R);
           }

           P = R * (RobotPosition - ForceFieldPosition);
           V = R * RobotVelocity;

           d = 0.0;
           if( abs(P(1,1)) >= ChannelWidth )
           {
               d = sgn(P(1,1)) * (abs(P(1,1)) - ChannelWidth);
           }
 
           // Calculate perpendicular (X) channel forces.
           if( d != 0.0 )
           {
               ForceFieldForces(1,1) = (RobotFieldConstants[0] * d) + (RobotFieldConstants[1] * V(1,1));
           }

           // Rotate back to original.
           ForceFieldForces = _R * ForceFieldForces;
           break;

        case FIELD_PMOVE :
           RobotPMoveUpdate(ForceFieldForces);
           break;

	case FIELD_2DSPRING :
           // constrain movement to (x,y) = (0,0) with a virtual spring

           P = RobotPosition - ForceFieldPosition;
           V = RobotVelocity;
	   ForceFieldForces = (RobotFieldConstants[0] * P) + (RobotFieldConstants[1] * V);
   	 

	/*
           dx = 0.0;
           dy = 0.0;
	   if( abs(P(1,1)) >= ChannelWidth )
           {
               dx = sgn(P(1,1)) * (abs(P(1,1)) - ChannelWidth);
           }
 	   
	   if( abs(P(2,1)) >= ChannelWidth )
           {
               dy = sgn(P(2,1)) * (abs(P(2,1)) - ChannelWidth);
           }
           // Calculate perpendicular (X) channel forces.
           if( dx != 0.0 )
           {
               ForceFieldForces(1,1) = (RobotFieldConstants[0] * dx) + (RobotFieldConstants[1] * V(1,1));
           }
 	   if( dy != 0.0 )
           {
               ForceFieldForces(2,1) = (RobotFieldConstants[0] * dy) + (RobotFieldConstants[1] * V(2,1));
           }

	*/

           break;

    }

    if (FieldType != FIELD_PMOVE)
    {
        SPMX_romxZ(D2R(SymmetryAxisAngle),R1);
        P1 = R1 * (RobotPosition - ViaPosition);
		
        // Is it a full (two-part) movement via central target?
        if( ContextFullMovementFlag[ContextType] )
        {
            // Movement to peripheral target; have we missed central target?
            if( (P1(2,1) >= MissedViaPointDistance) && (MissedViaPointDistance != 0.0) )
            {
                MissedViaPointFlag = TRUE;
            }
        }
        else
        {
            // Movement to central target only; have we moved too far?
            if( (P1(2,1) >= MovedTooFarDistance) && (MovedTooFarDistance != 0.0) )
            {
                MovedTooFarFlag = TRUE;
            }
        }
    }

    // Process Finite State Machine.
    StateProcessLoopTask();

    // Monitor timing of Forces Function (values saved to FrameData).
    ForcesFunctionLatency = RobotForcesFunctionLatency.After();

    ForceFieldRampValue = ForceFieldRamp.RampCurrent();
    RobotForces = (ForceFieldRampValue * ForceFieldForces);

    // Save frame data.
    FrameProcess();

    // Set forces to pass to robot API and clamp for safety.
    forces = RobotForces;
    forces.clampnorm(ForceMax);
}

/******************************************************************************/

void DeviceStop( void )
{
    // Stop and close robot.
    ROBOT_Stop(RobotID);
    ROBOT_SensorClose(RobotID);
    ROBOT_Close(RobotID);

    ForceFieldRamp.Stop();
    ChannelWidthRamp.Stop();

    RobotID = ROBOT_INVALID;
}

/******************************************************************************/

BOOL DeviceStart( void )
{
BOOL ok=TRUE;

    // Start the RAMPER object before starting the ROBOT; see hideous comment below.
    if( !(ForceFieldRamp.Start(ForceFieldRampTime) && ChannelWidthRamp.Start(ChannelWidthRampTime)) )
    {
        printf("Rampers failed to start.\n");
        ok = FALSE;
    }

    if( !ok )
    {
        DeviceStop();
        return(FALSE);
    }

    // For some hideous reason the RAMPER object starts at 1.0; JNI to investigate.
    ForceFieldRamp.Zero();

    // Open and start robot.
    if( (RobotID=ROBOT_Open(RobotName)) == ROBOT_INVALID )
    {
        printf("%s: Open failed.\n",RobotName);
        ok = FALSE;
    }
    else
    if( !RobotPMoveOpen() )
    {
        printf("PMove: Open failed.\n");
        ok = FALSE;
    }
    else
    if( !ROBOT_Start(RobotID,RobotForcesFunction) )
    {
        printf("%s: Start failed.\n",RobotName);
        ok = FALSE;
    }
    else
    if( !ROBOT_SensorOpen(RobotID) )
    {
        printf("%s: Cannot open sensor(s).\n",RobotName);
        ok = FALSE;
    }
    else
    if( RobotFT )
    {
        if( !ROBOT_SensorOpened_DAQFT() )
        {
            printf("%s: F/T sensor not opened.\n",RobotName);
            RobotFT = FALSE;
        }
    }

    if( !ok )
    {
        DeviceStop();
        return(FALSE);
    }

    printf("%s: Started.\n",RobotName);

    // Reset bias of F/T sensor if required.
    if( RobotFT )
    {
        printf("Press any key to reset bias of F/T sensor(s)...\n");
        while( !KB_anykey() );

        ok = ROBOT_SensorBiasReset_DAQFT(RobotID);
        printf("%s: F/T sensor bias reset: %s.\n",RobotName,STR_OkFailed(ok));
    }

    LoopTaskFrequency = ROBOT_LoopTaskGetFrequency(RobotID);
    LoopTaskPeriod = ROBOT_LoopTaskGetPeriod(RobotID);

    return(ok);
}

/******************************************************************************/

BOOL RobotActive( void )
{
BOOL flag=FALSE;

    if( ROBOT_Activated(RobotID) && ROBOT_Ramped(RobotID) )
    {
        flag = TRUE;
    }

    return(flag);
}

/******************************************************************************/

BOOL RobotHomeRectangle(  matrix &home, double xwid, double yhgt )
{
BOOL flag=FALSE;
double x,y;

    // Probably need to add an angle for rotated rectangle.
    x = abs(RobotPosition(1,1) - home(1,1));
    y = abs(RobotPosition(2,1) - home(2,1));

    flag = (x <= xwid) && (y <= yhgt);

    return(flag);
}

/******************************************************************************/

double RobotDistance( matrix &home )
{
double distance;

    distance = norm(RobotPosition-home);

    return(distance);
}

/******************************************************************************/

BOOL RobotHome( matrix &home, double tolerance )
{
BOOL flag=FALSE;

    if( RobotDistance(home) <= tolerance )
    {
        flag = TRUE;
    }

    return(flag);
}

/******************************************************************************/

BOOL RobotHome( void )
{
BOOL flag;

    flag = RobotHome(StartPosition,StartTolerance);

    return(flag);
}

/******************************************************************************/

BOOL RobotInsideVia( void )
{
BOOL flag=FALSE;

    switch( ViaType )
    {
        case VIA_CIRCLE :
            flag = RobotHome(ViaPosition,ViaRadius);
            break;

        case VIA_RECTANGLE :
            flag = RobotHomeRectangle(ViaPosition,ViaWidth,ViaHeight);
            break;
    }

    return(flag);
}

/******************************************************************************/

BOOL MovementStarted( void )
{
BOOL flag;

    flag = !RobotHome(StartPosition,StartTolerance);

    return(flag);
}

/******************************************************************************/

BOOL MovementFinished( void )
{
BOOL flag=FALSE;

    if( FieldType == FIELD_PMOVE )
    {
        flag = RobotPMoveFinished();
        return(flag);
    }

    // Is robot in the finish position?
    if( !RobotHome(FinishPosition,FinishTolerance) )
    {
        MovementFinishedTimer.Reset();    
    }
     
    // Has the robot been in the finish position for the required amount of time?
    if( MovementFinishedTimer.ExpiredSeconds(FinishToleranceTime) )
    {
        flag = TRUE;
    }
	
    return(flag);
}

/******************************************************************************/

BOOL RobotNotMoving( void )
{
BOOL flag;

    if( NotMovingSpeed == 0.0 )
    {
        return(TRUE);
    }

    if( RobotSpeed > NotMovingSpeed )
    {
        NotMovingTimer.Reset();
    }

    flag = NotMovingTimer.ExpiredSeconds(NotMovingTime);

    return(flag);
}

/******************************************************************************/

void StateNext( int state )
{
    if( State == state )
    {
        return;
    }

    printf("STATE: %s[%d] > %s[%d] (%.0lf msec).\n",StateText[State],State,StateText[state],state,StateTimer.Elapsed());
    StateTimer.Reset();
    StateFirstFlag = TRUE;
    StateLast = State;
    State = state;
}

/******************************************************************************/

BOOL StateFirst( void )
{
BOOL flag;

    flag = StateFirstFlag;
    StateFirstFlag = FALSE;

    return(flag);
}

/******************************************************************************/

void StateGraphicsNext( int state )
{
    if( StateGraphics == state )
    {
        return;
    }

    StateGraphicsTimer.Reset();
    StateGraphicsLast = StateGraphics;
    StateGraphics = state;

    // The visual target first appears in this state, so set graphics sync timer.
    if( StateGraphics == STATE_GO )
    {
        // Set graphics sync timer relative to offset of next vertical retrace.
        GraphicsTargetTimer.Reset(-GRAPHICS_VerticalRetraceOffsetTimeUntilNext());
    }
}

/******************************************************************************/

void TrialSetup( void )
{
int i;

    // Load trial variables from TrialData.
    TrialData.RowLoad(Trial);

    // Make sure the force ramper is zero before starting the trial.
    //ForceFieldRamp.Zero();
    ChannelWidthRamp.One();

    // Set robot force field variables.
    if( FieldType != FIELD_SAMEASLAST )
    {
        RobotFieldType = FieldType;

        for( i=0; (i < FIELD_CONSTANTS); i++ )
        {
            RobotFieldConstants[i] = FieldConstants[i];
        }

        RobotFieldAngle = FieldAngle;
        SPMX_romxZ(D2R(RobotFieldAngle),RobotFieldMatrix);
    }

    ViaEntryPosition(1,1) = ViaRadius * sin(D2R(ViaEntryAngle));
    ViaEntryPosition(2,1) = -ViaRadius * cos(D2R(ViaEntryAngle));

    TrialRunning = FALSE;

    StateGraphicsNext(State);

    printf("TrialSetup: Trial=%d FieldType=%d ContextType=%d MoveOrderType=%d\n",Trial,FieldType,ContextType,MovementOrderType);
}

/******************************************************************************/

void TrialStart( void )
{
    printf("Starting Trial %d...\n",Trial);
    printf("TargetAngle=%.1lf(deg) Phase=%d Field=%d FieldConstant=%.2lf,%.2lf\n",TargetAngle,TrialPhase,FieldType,FieldConstants[0],FieldConstants[1]);
    disp(StartPosition);
    disp(FinishPosition);

    TrialTimer.Reset();
    TrialTime = TrialTimer.ElapsedSeconds();
    TrialRunning = TRUE;

    MovedTooFarFlag = FALSE;
    MissedViaPointFlag = FALSE; 
    TargetResolveFlag = FALSE;
    ViaNotMovingTime = 0.0;

    // Start force-field if the channel/field is on the first movement (or PMove).
    if( (ChannelOrderType == CHANNEL_FIRST) || (FieldType == FIELD_PMOVE) )
    {
        ForceFieldStart();
    }

    // Start recording frame data for trial.
    FrameStart();
}

/******************************************************************************/

void TrialStop( void )
{
    TrialRunning = FALSE;
    printf("Stopping Trial %d...\n",Trial);

    // Stop recording frame for trial.
    FrameStop();

    // Stop force field.
    ForceFieldStop();

    TrialDuration = TrialTimer.ElapsedSeconds();
    InterTrialDelayTimer.Reset();
}

/******************************************************************************/

void TrialAbort( void )
{
    TrialRunning = FALSE;
    printf("Aborting Trial %d...\n",Trial);

    // Stop recording frame data for trial.
    FrameStop();

    // Stop force field.
    ForceFieldStop();
}

/******************************************************************************/

BOOL TrialSave( void )
{
BOOL ok=FALSE;
int i;

    ExperimentTime = ExperimentTimer.ElapsedSeconds();
    MissTrials = MissTrialsTotal;
    for( i=0; (i < MISS_TRIAL_TYPES); i++ )
    {
        MissTrialsType[i] = MissTrialsTypeTotal[i];
    }

    // Put values in the trial data
    TrialData.RowSave(Trial);

    // Set-up the data file on the first trial.
    if( Trial == 1 )
    {
        // Open the file for trial data.
        if( !DATAFILE_Open(DataFile,TrialData,FrameData) )
        {
            printf("DATAFILE: Cannot open file: %s\n",DataFile);
            return(FALSE);
        }
    }

    // Write the trial data to the file.
    printf("Saving trial %d: %d frames of data collected in %.2lf seconds.\n",Trial,FrameData.GetRow(),TrialDuration);
    ok = DATAFILE_TrialSave(Trial);
    printf("%s %s Trial=%d.\n",DataFile,STR_OkFailed(ok),Trial);

    return(ok);
}

/******************************************************************************/

void TrialExit( void )
{
    // Close the data file if it has been opened.
    if( DATAFILE_Opened() )
    {
        if( !DATAFILE_Close() )
        {
            printf("Please resolve problems with data file: %s\n",DataFile);
        }
    }
}

/******************************************************************************/

BOOL TrialNext( void )
{
BOOL flag=FALSE;

    if( Trial < Trials )
    {
        Trial++;
        flag = TRUE;
    }

    return(flag);
}

/******************************************************************************/

void BeepGo( void )
{
    WaveListPlay("HighBip");
}

/******************************************************************************/

void BeepError( void )
{
    WaveListPlay("LowBip");
}

/******************************************************************************/

void MessageClear( void )
{
    GraphicsText("");
    GRAPHICS_ClearColor(); // Default background color.
}

/******************************************************************************/

void MessageSet( char *text, int background )
{
    MessageClear();
    
    GraphicsText(text);

    if( background != -1 )
    {
        GRAPHICS_ClearColor(background);
    }
}

/******************************************************************************/

void MessageSet( char *text )
{
int background=-1;

    MessageSet(text,background);
}

/******************************************************************************/

void ErrorFrameDataFull( void )
{
    MessageSet("Frame data full",GraphicsBackGround);
    printf("Error: Frame data full\n");
}

/******************************************************************************/

void ErrorMessage( char *str )
{
    MessageSet(str,GraphicsBackGround);
    printf("Error: %s\n",str);
}

/******************************************************************************/

void ErrorMoveWaitTimeOut( void )
{
    MessageSet("Move After Beep",GraphicsBackGround);
    printf("Error: MoveWaitTimeOut\n");
}

/******************************************************************************/

void ErrorMoveTooSlow( void )
{
    MessageSet("Too Slow");
    printf("Error: TooSlow\n");
}

/******************************************************************************/

void ErrorMoveTimeOut( void )
{
    MessageSet("Too Slow",GraphicsBackGround);
    printf("Error: MoveTimeOut\n");
}

/******************************************************************************/

void ErrorMissedVia( void )
{
    MessageSet("Missed Central Target",GraphicsBackGround);
    printf("Error: MissedVia\n");
}

/******************************************************************************/

void ErrorMoveTooSoon( void )
{
    MessageSet("Moved Too Soon",GraphicsBackGround);
    printf("Error: MoveTooSoon\n");
}

/******************************************************************************/

void ErrorViaTooLong( void )
{
    MessageSet("Too Long at CT",GraphicsBackGround);
    printf("Error: ViaTooLong\n");
}

/******************************************************************************/

void ErrorViaEntry( void )
{
    MessageSet("Missed Target Entrance",GraphicsBackGround);
    printf("Error: ViaEntry\n");
}

/******************************************************************************/

void ErrorMovedTooFar( void )
{
    MessageSet("Moved Too Far",GraphicsBackGround);
    printf("Error: MovedTooFar\n");
}

/******************************************************************************/

void ErrorViaTooShort( void )
{
    MessageSet("Slow Down at CT",GraphicsBackGround);
    printf("Error: ViaTooShort\n");
}

/******************************************************************************/

void ErrorRobotInactive( void )
{
    MessageSet("Handle Switch",GraphicsBackGround);
    printf("Error: RobotInactive\n");
}

/******************************************************************************/

void ErrorState( int state )
{
    BeepError();
    StateErrorResume = state;
    StateNext(STATE_ERROR);
}

/******************************************************************************/

void ErrorResume( void )
{
    MessageClear();
    StateNext(StateErrorResume);
}

/******************************************************************************/

void MissTrial( int type )
{
int i;

    MissTrialsTotal++;
    MissTrialsTypeTotal[type]++;

    MissTrialsPercent = 100.0 * ((double)MissTrialsTotal / (double)Trial);
    printf("\nMiss Trials = %d/%d (%.0lf%%) [Type=%d]\n\n",MissTrialsTotal,Trial,MissTrialsPercent,type);
    for( i=0; (i < MISS_TRIAL_TYPES); i++ )
    {
        printf("MissTrialsTypeTotal[%02d]=%d\n",i,MissTrialsTypeTotal[i]);
    }

    ErrorState(STATE_SETUP);
}

/******************************************************************************/

void FeedbackMessage( void )
{
    if( FieldType == FIELD_PMOVE )
    {
        return;
    }

    // Duration criteria for feedback, and positive feedback

    if( ContextFullMovementFlag[ContextType] || ContextSingleMovementFlag[ContextType] )
    {   // It's a full (two-part) movement.
        // Check timing of the first (STATE_MOVING0) and second (STATE_MOVING1) movements.
      	if( ((MovementFirstTime >= MovementFirstTooSlow) && (MovementFirstTooSlow != 0.0)) || ((MovementSecondTime >= MovementSecondTooSlow) && (MovementSecondTooSlow != 0.0))  )
      	{
      	    GraphicsText("Too Slow");                
      	}
      	else 
      	if( ((MovementFirstTime <= MovementFirstTooFast) && (MovementFirstTooFast != 0.0)) || ((MovementSecondTime <= MovementSecondTooFast) && (MovementSecondTooFast != 0.0)))
        {
            GraphicsText("Too Fast");                
        }
        else
        {
            GraphicsText("Correct Speed");
        }
    }
    else
    {   // It's a single movement only to the central target.
        if( MovedTooFarFlag )
        {
            BeepError();
            ErrorMovedTooFar();
        }
    }

    printf("MovementFirstTime=%.3lf (Slow=%.3lf,Fast=%.3lf)\n",MovementFirstTime,MovementFirstTooSlow,MovementFirstTooFast);
    printf("MovementSecondTime=%.3lf (Slow=%.3lf,Fast=%.3lf)\n",MovementSecondTime,MovementSecondTooSlow,MovementSecondTooFast);
	
}

/******************************************************************************/

void ViaPointEntered( BOOL &ViaEnteredFlag, BOOL &ViaErrorFlag )
{
double via_y,via_x;

    via_y = ViaPosition(2,1) - (ViaHeight/2.0);

    if( RobotPosition(2,1) >= via_y )
    {
        via_x = abs(RobotPosition(1,1) - ViaPosition(1,1));

        if( via_x <= (CursorRadius+(ViaWidth/2.0)) )
        {
            ViaEnteredFlag = TRUE;
        }
        else
        {
            ViaErrorFlag = TRUE;
        }
    }
}

/******************************************************************************/

BOOL ViaEntryAngleMissTrial( void )
{
BOOL flag=FALSE;

    if( RobotHome(ViaPosition,ViaRadius) && (MovementOrderType == ORDER_LEAD_IN) )
    {
        if( abs(RobotPosition(1,1)-ViaPosition(1,1)) > (ViaEntryPosition(1,1)-(CursorRadius*cos(D2R(ViaEntryAngle)))) )
        {
            flag = TRUE;
            ErrorViaEntry();
            TrialAbort(); // Abort the current trial.
            MissTrial(MISS_TRIAL_VIAENTRY);  // Generate miss trial.
        }
    }

    return(flag);
}

/******************************************************************************/

void StateProcessLoopTask( void )
{
//BOOL ViaEnteredFlag=FALSE,ViaErrorFlag=FALSE;

    // Only some states are processing in the LoopTask.
    if( !StateLoopTask[State] )
    {
        return;
    }

    // State processing.
    switch( State )
    {
        case STATE_MOVEWAIT :
           if( ( MovementStarted() || (FieldType == FIELD_PMOVE) ) || (ContextType == PASSIVE_WAIT) )
           {
               MovementDurationTimer.Reset();
               MovementFirstTimer.Reset();
               MovementReactionTime = MovementReactionTimer.ElapsedSeconds();
               StateNext(STATE_MOVING0);
               break;
           }

           if( MovementReactionTimer.ExpiredSeconds(MovementReactionTimeOut) )
           {
               StateNext(STATE_TIMEOUT);
               break;
           }
           break;

        case STATE_MOVING0 : 
            if ( FieldType == FIELD_PMOVE )
            {
                if ( RobotPMoveFinished() )
                {
                     MovementDurationTime = MovementDurationTimer.ElapsedSeconds();
                     StateNext(STATE_FINISH);
                     break;
                }

                break;
            }
			
			if ( ContextType == PASSIVE_WAIT )
			{
			    MovementDurationTime = MovementDurationTimer.ElapsedSeconds();
                PostMoveDelayTime = ContextFullMovementTimeData.Mean() - MovementDurationTime;
			    StateNext(STATE_POSTMOVEDELAY);
				break;
			}

            if( MovementDurationTimer.ExpiredSeconds(MovementDurationTimeOut) )
            {
                StateNext(STATE_TIMEOUT);
                break;
            }

            
            // Have we entered the via-point at the correct angle?
            /*if( ViaEntryAngleMissTrial() )
            {
                break;
            }*/

            /*ViaPointEntered(ViaEnteredFlag,ViaErrorFlag);

            if( ViaErrorFlag )
            {
                ErrorViaEntry();
                TrialAbort(); // Abort the current trial.
                MissTrial(MISS_TRIAL_VIAENTRY);  // Generate miss trial.
            }*/

            if( ContextFullMovementFlag[ContextType] )
            {
                // It's a full (two-part) movement, so have we entered the via-point.
                if( RobotInsideVia() )
                {
                    MovementFirstTime = MovementFirstTimer.ElapsedSeconds();
                    PassingViaTimer.Reset();

                    // Stop the force-field if it's a follow-through paradigm.
                    if( ChannelOrderType == CHANNEL_FIRST )
                    {		
                        ForceFieldStop();		
                    }

                    /*// Start force-field if it's a lead-in paradigm.
                    if( MovementOrderType == ORDER_LEAD_IN )
                    {
                        ForceFieldStart();
                    }*/

                    StateNext(STATE_VIAPOINT);
                    break;
                }

                if( MissedViaPointFlag )
                {
                    ErrorMissedVia();
                    TrialAbort(); // Abort the current trial.
                    MissTrial(MISS_TRIAL_MISSEDVIA);  // Generate miss trial.
                }
            }
            else
            {
                // It's a single movement only to the central target...
                if( MovementFinished() )
                {
                    MovementFirstTime = MovementFirstTimer.ElapsedSeconds();                
                    MovementDurationTime = MovementDurationTimer.ElapsedSeconds();
                    PostMoveDelayTime = ContextFullMovementTimeData.Mean() - MovementDurationTime;

                    // Stop the force-field if it's a follow-through paradigm.
                    if( ChannelOrderType == CHANNEL_FIRST )
                    {		
                        ForceFieldStop();		
                    }

                    StateNext((MovementOrderType == ORDER_SINGLE_MOVEMENT) ? STATE_FINISH : STATE_POSTMOVEDELAY);
                    break;
                }
            }
            break;

        case STATE_VIAPOINT :
            if( MovementDurationTimer.ExpiredSeconds(MovementDurationTimeOut) )
            {
                StateNext(STATE_TIMEOUT);
                break;
            }

            // Too long in the via point?
            if( (ViaNotMovingTime >= ViaTimeOutTime) && (ViaTimeOutTime != 0.0) )
            {
                ErrorViaTooLong();
                TrialAbort(); // Abort the current trial.
                MissTrial(MISS_TRIAL_VIATOOLONG);  // Generate miss trial.
                break;
            }

            if( ViaNotMovingTime == 0.0 )
            {
                if( RobotSpeed >= ViaNotMovingSpeed )
                {
                    ViaNotMovingTimer.Reset();
                }
            }

            if( ViaNotMovingTimer.ExpiredSeconds(ViaToleranceTime) )
            {
                ViaNotMovingTime = ViaNotMovingTimer.ElapsedSeconds();
            }

             // Start force-field if the channel/field is on the second movement
            if( (ViaNotMovingTime >= ViaToleranceTime) && (ChannelOrderType == CHANNEL_SECOND) && !ForceFieldStarted )
            {
                ForceFieldStart();
            }

            // It's a full (two-part) movement...
            if( !RobotInsideVia() )
            {
                // We've left the via point...
                PassingViaTime = PassingViaTimer.ElapsedSeconds();

                // Check if left via point too early or too fast, and if not then move to next state STATE_MOVING1
                if( ViaNotMovingTime < ViaToleranceTime )
                {
                    printf("\nPassingViaTime=%0.2lf, ViaNotMovingTime=%.02lf, ViaToleranceTime=%0.2lf\n",PassingViaTime,ViaNotMovingTime,ViaToleranceTime);
                    ErrorViaTooShort();
                    TrialAbort(); // Abort the current trial
                    MissTrial(MISS_TRIAL_VIATOOSHORT);  // Generate miss trial
                    break;
                }

                // We've been in the via-point for the right amoung of time.
                printf("\nPassingViaTime=%0.2lf, ViaNotMovingTime=%.02lf, ViaToleranceTime=%0.2lf(sec)\n",PassingViaTime,ViaNotMovingTime,ViaToleranceTime);
                MovementSecondTimer.Reset();

                /*// Start force-field if it's a lead-in paradigm.
                if( MovementOrderType == ORDER_LEAD_IN )
                {
                    ForceFieldStart();
                }*/

                StateNext(STATE_MOVING1);
            }
            break;

        case STATE_MOVING1 :
            if( MovementFinished() )
            {
                MovementSecondTime = MovementSecondTimer.ElapsedSeconds();
                MovementDurationTime = MovementDurationTimer.ElapsedSeconds();

                ContextFullMovementTimeData.Data(MovementDurationTime);

                // Stop force-field if the channel/field is on the second movement.
                if( ChannelOrderType == CHANNEL_SECOND )
                {
                    ForceFieldStop();
                }

                StateNext(STATE_FINISH);
                break;
            }
           
            if( MovementDurationTimer.ExpiredSeconds(MovementDurationTimeOut) )
            {
                StateNext(STATE_TIMEOUT);
                break;
            }
            break;

        case STATE_POSTMOVEDELAY :
            if( StateTimer.ExpiredSeconds(PostMoveDelayTime) )
            {
                StateNext(STATE_FINISH);
                break;
            }
            break;
		
    }
}

/******************************************************************************/

void StateProcess( void )
{
    // Check that robot is in a safe state.
    if( !ROBOT_Safe(ROBOT_ID) )
    {
        printf("Robot not safe.\n");
        ProgramExit();
    }

    // Special processing while a trial is running.
    if( TrialRunning )
    {
        if( !RobotActive() )
        {
            // If robot is not active, abort current trial.
            ErrorRobotInactive();
            TrialAbort();
            MissTrial(MISS_TRIAL_ROBOTINACTIVE);
        }
        else
        if( FrameData.Full() )
        {
            // Abort current trial if frame data is full.
            ErrorFrameDataFull();
            TrialAbort();
            MissTrial(MISS_TRIAL_FRAMEDATAFULL);
        }
    }

    // Some states are processing in the LoopTask.
    if( StateLoopTask[State] )
    {    
        return;
    }

    // State processing.
    switch( State )
    {
        case STATE_INITIALIZE :
           // Initialization state.
           ExperimentTimer.Reset();
           StateNext(STATE_SETUP);
           break;

        case STATE_SETUP :
           // Setup details of next trial, but only when robot stationary and active.
           
           if( !(RobotNotMoving() && RobotActive()) )
           {
               break;
           }

           TrialSetup();

           if( FieldType == FIELD_PMOVE )
           {
               // No point doing a passive-return movement on the last trial.
               if( Trial == Trials )
               {
                   StateNext(STATE_EXIT);
                   break;
               }
           }

           StateNext(STATE_HOME);
           break;

        case STATE_HOME :
           // Start trial when robot in home position (and stationary and active).
           if (FieldType == FIELD_PMOVE)
           {
               StateNext(STATE_START);
	   }

	   if( RobotNotMoving() && RobotHome() && RobotActive() )
           {
               StateNext(STATE_START);
               break;
           }
           break;

        case STATE_START :
           // Start trial.
           TrialStart();

           if( FieldType == FIELD_PMOVE )
           {
               StateNext(STATE_MOVEWAIT);
           }
           else
           {
               StateNext(STATE_DELAY);
           }
           break;

        case STATE_DELAY :
           // Delay period before go signal.
           if( (FieldType == FIELD_SAMEASLAST) && (ContextType == PASSIVE_MOVE) )
           {
               if( StateTimer.ExpiredSeconds(PMoveMovementTime+PMoveHoldTime+PMoveRampTime) )
               {
                   StateNext(STATE_FINISH);
               }
               break;
           }

           if( StateTimer.ExpiredSeconds(TrialDelay) )
           {
               StateNext(STATE_GO);
               break;
           }

           if( MovementStarted() )
           {
               ErrorMoveTooSoon();
               TrialAbort();
               MissTrial(MISS_TRIAL_MOVETOOSOON);
           }
           break;

        case STATE_GO :
           // Go signal to cue movement.
           if ( ContextType != PASSIVE_WAIT )
		   {
		      BeepGo();
		   }
		   MovementReactionTimer.Reset();
           StateNext(STATE_MOVEWAIT);
           break;

        case STATE_MOVEWAIT :
           // Process in the robot forces function (LoopTask)
           break;

        case STATE_MOVING0 :
           // Process in the robot forces function (LoopTask)
           break;

        case STATE_VIAPOINT :
           // Process in the robot forces function (LoopTask)
           break;

        case STATE_MOVING1 :
            // Process in the robot forces function (LoopTask)
            break;

        case STATE_FINISH :
           // Trial has finished so stop trial.
           TrialStop();

           // Save the data for this trial.
           if( !TrialSave() )
           {
               printf("Cannot save Trial %d.\n",Trial);
               StateNext(STATE_EXIT);
               break;
           }

           StateNext(STATE_FEEDBACK);
           break;
    
        case STATE_FEEDBACK :
           if( StateFirst() && ContextType != PASSIVE_WAIT )
           {
	       // Display feedback on movement speeds to subject
	          FeedbackMessage();
           }

	   if( StateTimer.ExpiredSeconds(FeedbackTime) || (FieldType == FIELD_PMOVE) )
	   {		
               if( RestBreakNow() )
               {
                   MessageClear();
                   StateNext(STATE_REST);
                   break;
               }

               StateNext(STATE_NEXT);
           }
           break;

        case STATE_NEXT :
           if( !TrialNext() )
           {
               StateNext(STATE_EXIT);
               break;
           }

           if( (StateLast == STATE_REST) && !RobotActive() )
           {
               TrialSetup();
           }

           StateNext(STATE_INTERTRIAL);
           break;

        case STATE_INTERTRIAL :
           // Wait for the intertrial delay to expire.
           if( !InterTrialDelayTimer.ExpiredSeconds(InterTrialDelay) )
           {
               break;
           }

           MessageClear();

           StateNext(STATE_SETUP);
           break;

        case STATE_EXIT :
           if( StateFirst() )
           {
               ExperimentSeconds = ExperimentTimer.ElapsedSeconds();
               ExperimentMinutes = ExperimentSeconds / 60.0;
               GraphicsText(STR_stringf("Game Over (%.1lf minutes)",ExperimentMinutes));
           }
           break;

        case STATE_TIMEOUT :
           switch( StateLast ) // Which state had the timeout?
           {
               case STATE_MOVEWAIT :
                    ErrorMoveWaitTimeOut();
                    break;
 
               case STATE_MOVING0 :
                    ErrorMoveTooSlow();  
                    break;

               case STATE_VIAPOINT :
                    ErrorViaTooLong();
                    break;

               case STATE_MOVING1 :
                    ErrorMoveTimeOut();
                    break;
                   
               default :
                   ErrorMessage(STR_stringf("%s TimeOut",StateText[StateLast]));
                   break;
           }

           TrialAbort(); // Abort the current trial.
           MissTrial(MISS_TRIAL_TIMEOUT);  // Generate miss trial.
           break;

        case STATE_ERROR :
           if( StateTimer.ExpiredSeconds(ErrorWait) )
           {
               ErrorResume();
           }
           break;

        case STATE_REST :
           RestBreakRemainSeconds = (RestBreakSeconds - StateTimer.ElapsedSeconds());
           RestBreakRemainPercent = (RestBreakRemainSeconds / RestBreakSeconds);

           if( RestBreakRemainSeconds <= 0.0 )
           {
               StateNext(STATE_NEXT);
           }
           break;
    }
}

/******************************************************************************/

void GraphicsResults( void )
{
    printf("----------------------------------------------------------------\n");

    printf("Monitor Refresh Rate %.0lf Hz.\n",GRAPHICS_VerticalRetraceFrequency);
    GraphicsDisplayFrequency.Results();
    GraphicsDisplayLatency.Results();
    GraphicsSwapBufferLatency.Results();
    GraphicsClearStereoLatency.Results();
    GraphicsClearMonoLatency.Results();
    GraphicsIdleFrequency.Results();

    if( GraphicsVerticalRetraceSyncTime != 0.0 )
    {
        GRAPHICS_VerticalRetraceResults();
    }

    printf("----------------------------------------------------------------\n");
}

/******************************************************************************/

void Results( void )
{
    // Print results for various timers and things.
    RobotForcesFunctionLatency.Results();
    RobotForcesFunctionFrequency.Results();
    GraphicsResults();
    WaveListPlayInterval.Results();
    ContextFullMovementTimeData.Results();
}

/******************************************************************************/

void ProgramExit( void )
{
    // Do trial data exit stuff.
    TrialExit();

    // Stop, close and other final stuff.
    DeviceStop();
    GRAPHICS_Stop();
    Results();
    WAVELIST_Close(WaveList);

    printf("ExperimentTime = %.0lf minutes.\n",ExperimentTimer.ElapsedMinutes());

    // Exit the program.
    exit(0);
}

/******************************************************************************/

matrix TargetAngleVector( double angle, double distance )
{
static matrix vector(3,1);

   // Create a vector for target angle.
   vector(1,1) = distance * sin(D2R(angle));
   vector(2,1) = distance * cos(D2R(angle));
   vector(3,1) = 0.0;

   return(vector);
}

/******************************************************************************/

void GraphicsDisplayCursor( void )
{
static matrix posn(3,1);

    posn = CursorPosition;
    posn(3,1) = 1.0;
    GRAPHICS_Circle(&posn,CursorRadius,CursorColor);

}

/******************************************************************************/

void GraphicsDisplayRing( matrix *posn, double radius, double width, int color )
{
static matrix P(3,1);

    P.zeros();
    if( posn != NULL )
    {
        P = *posn;
    }

    GRAPHICS_Circle(&P,radius,color);
    P(3,1) += 0.1;
    GRAPHICS_Circle(&P,radius-width,GraphicsBackGround);
}

/******************************************************************************/

void GraphicsDisplayTarget( void )
{
static matrix posn(3,1),R(3,3),S(3,1),E(3,1);
static matrix StartToCentralVector(3,1);
double StartToCentralDistance;
BOOL DisplayTargetFlag;
BOOL FilledTargetFlag;
BOOL DualTargetFlag;
int attr;
double i;

    // The following code was in GraphicsDisplay()...

   
    if( (FieldType == FIELD_PMOVE) || (ContextType == PASSIVE_MOVE) )
	{
        // Deliberate mixture of finish and start variables for passive-return trials.
        attr = RobotHome(FinishPosition,StartTolerance) ? StartColor : NotStartColor;	    

        posn = FinishPosition;
        posn(3,1) = 0.0;

        GRAPHICS_Circle(&posn,StartRadius,attr);

        return;
    }

    // Display via point for movement.
    posn = ViaPosition;
    posn(3,1) = 0.0;
    if ( ContextType != PASSIVE_WAIT ) 
  	{
       switch( ViaType )
       {
           case VIA_CIRCLE :
               GRAPHICS_Circle(&posn,ViaRadius,ViaColor);
               break;

           case VIA_RECTANGLE :
               GRAPHICS_Rectangle(&posn,ViaWidth,ViaHeight,ViaColor);
               break;
       }
	}

    /*if( MovementOrderType == ORDER_LEAD_IN )
    {
        for( i=-1.0; (i <= 1.0); i+=2.0 )
        {
            S(1,1) = i * (ViaRadius-(ViaEntryLineLength/2.0)) * sin(D2R(ViaEntryAngle));
            S(2,1) = -(ViaRadius-(ViaEntryLineLength/2.0)) * cos(D2R(ViaEntryAngle));
            S += ViaPosition;
            S(3,1) = 0.5;

            E(1,1) = i * (ViaRadius+(ViaEntryLineLength/2.0)) * sin(D2R(ViaEntryAngle));
            E(2,1) = -(ViaRadius+(ViaEntryLineLength/2.0)) * cos(D2R(ViaEntryAngle));
            E += ViaPosition;
            E(3,1) = 0.5;

            GRAPHICS_Line(S,E,ViaEntryLineWidth,WHITE);
        }
    }*/

    switch( MovementOrderType )
    {
        case ORDER_FOLLOW_THROUGH :
            SPMX_romxZ(D2R(SymmetryAxisAngle),R);
            break;

        case ORDER_LEAD_IN :
            SPMX_romxZ(D2R(SymmetryAxisAngle+TargetAngle),R);
            break;

        case ORDER_SINGLE_MOVEMENT :
            // This isn't used in current paradigm so hasnt' been tested.
            SPMX_romxZ(D2R(SymmetryAxisAngle+TargetAngle),R);
            break;
    }

    StartToCentralVector = RobotPosition - StartPosition;
    StartToCentralVector = R * StartToCentralVector;
    StartToCentralDistance = StartToCentralVector(2,1);

    DisplayTargetFlag = FALSE;
    FilledTargetFlag = TRUE;
    DualTargetFlag = FALSE;

    switch( ContextType )
    {
        case TARGET_STATIC_ON :
            DisplayTargetFlag = TRUE;
            break;

        case TARGET_APPEAR :
            if( !TargetResolveFlag )
            {
                TargetResolveFlag = (StartToCentralDistance > TargetResolveDistance);
            }

            DisplayTargetFlag = TargetResolveFlag;
            break;

        case TARGET_STOP :
            if( !TargetResolveFlag )
            {
                TargetResolveFlag = (StartToCentralDistance > TargetResolveDistance);
            }

            DisplayTargetFlag = !TargetResolveFlag;
            break;

        case TARGET_CENTRAL_ONLY :
            DisplayTargetFlag = FALSE;
            break;
            
            
        case TARGET_VISUAL_ONLY :
            DisplayTargetFlag = TRUE;
            break;

        case TARGET_DUAL_OFF :
            if( !TargetResolveFlag )
            {
                TargetResolveFlag = (StartToCentralDistance > TargetResolveDistance);
            }

            DisplayTargetFlag = !TargetResolveFlag;
            DualTargetFlag = DisplayTargetFlag;
            break;

        case DUAL_PLANNING :
            if( !TargetResolveFlag )
            {
                TargetResolveFlag = (StartToCentralDistance > TargetResolveDistance);
            }

            DisplayTargetFlag = TRUE;
            DualTargetFlag = !TargetResolveFlag;
            break;

        case TARGET_STATIC_GO :
            DisplayTargetFlag = TRUE;
            FilledTargetFlag = (StateGraphics >= STATE_GO);
            break;

        case DUAL_PLANNING_GO :
            DisplayTargetFlag = TRUE;
            FilledTargetFlag = (StateGraphics >= STATE_GO);
            DualTargetFlag = TRUE;
            break;

		case PASSIVE_WAIT :
			//show only the home position
			break;
    }

    if( DisplayTargetFlag )
    {
        posn = TargetPosition;

        if( FilledTargetFlag )
        {
            GRAPHICS_Circle(&posn,TargetRadius,TargetColor);
        }
        else
        {
            GraphicsDisplayRing(&posn,TargetRadius,TargetOutlineWidth,TargetColor);
        }
    }

    if( DualTargetFlag )
    {
        posn = ViaPosition + TargetAngleVector(SymmetryAxisAngle-TargetAngle,MovementSecondDistance);

        if( FilledTargetFlag && (ContextType != DUAL_PLANNING_GO) )
        {
            GRAPHICS_Circle(&posn,TargetRadius,TargetColor);
        }
        else
        {
             GraphicsDisplayRing(&posn,TargetRadius,TargetOutlineWidth,TargetColor);
        }
    }
}

/******************************************************************************/

void GraphicsDisplayTeaPot( void )
{
    glPushMatrix();
    GRAPHICS_ColorSet(WHITE);
    glRotated(StateTimer.ElapsedSeconds() * TeapotRotateSpeed,1.0,1.0,1.0);
    glLineWidth(1.0);
    glutWireTeapot(TeapotSize * RestBreakRemainPercent);
    glPopMatrix();
}

/******************************************************************************/

void GraphicsDisplay( void )
{
int attr;
static matrix posn;

    // Mark time before we start drawing the graphics scene.
    GraphicsDisplayLatency.Before();

    // Clear "stereo" graphics buffers.
    GraphicsClearStereoLatency.Before();
    GRAPHICS_ClearStereo();
    GraphicsClearStereoLatency.After();

    // Loop for each eye (stereo 3D).
    GRAPHICS_EyeLoop(eye)
    {
        // Set view for each eye (stereo 3D).
        GRAPHICS_ViewCalib(eye);

        // Clear "mono" graphics buffers.
        GraphicsClearMonoLatency.Before();
        GRAPHICS_ClearMono();
        GraphicsClearMonoLatency.After();

        // Display text.
        GraphicsDisplayText();

        // Display rotating teapot during rest period.
        if( StateGraphics == STATE_REST )
        {
            GraphicsDisplayTeaPot();
            continue;
        }

        // Display home position at start of trial.
        if( (StateGraphics >= STATE_SETUP) && (StateGraphics <= STATE_INTERTRIAL) && (FieldType != FIELD_PMOVE) )
        {
            attr = RobotHome() ? StartColor : NotStartColor;

            posn = StartPosition;
            posn(3,1) = 0.0;

            GRAPHICS_Circle(&posn,StartRadius,attr);
        }

        // Display targets when trial running.
        if( (StateGraphics >= STATE_START) && (StateGraphics <= STATE_INTERTRIAL) )
        {
            // Display target for movement.
            GraphicsDisplayTarget();
        }

        // Display robot position cursor.
        if( StateGraphics != STATE_ERROR )
        {
            GraphicsDisplayCursor();
        }
    }

    // Mark time now that scene has been drawn.
    GraphicsDisplayLatency.After();

    // Display the graphics buffer we've just drawn.
    GraphicsSwapBufferLatency.Before();
    GRAPHICS_SwapBuffers();
    GraphicsSwapBufferLatency.After();

    // Mark time for display frequency.
    GraphicsDisplayFrequency.Loop(); 
}

/******************************************************************************/

void GraphicsIdle( void )
{
BOOL draw=FALSE;

    GraphicsIdleFrequency.Loop();

    // Process Finite State Machine.
    StateProcess();

    // If set, graphics frames are timed to occur just before next vertical retrace.
    if( GraphicsVerticalRetraceSyncTime != 0.0 )
    {
        // This function catches the vertical retrace, resetting a timer.
        GRAPHICS_VerticalRetraceCatch(GraphicsVerticalRetraceCatchTime);

        // Set draw flag if next vertical retrace is about to occur.
        if( GRAPHICS_VerticalRetraceOnsetTimeUntilNext() <= seconds2milliseconds(GraphicsVerticalRetraceSyncTime) )
        {
            // But make sure we haven't already drawn a frame in this vertical retrace cycle.
            if( GraphicsDisplayFrequency.ElapsedSeconds() >= (GRAPHICS_VerticalRetracePeriod/2.0) )
            {
                draw = TRUE;
            }
        }
    }
    else
    {
        draw = TRUE;
    }

    // Draw graphics frame only if draw flag set.
    if( draw )
    {
        StateGraphicsNext(State); // Safe to set graphics state at this point.
        GraphicsDisplay();
    }

    Sleep(0);
}

/******************************************************************************/

void GraphicsKeyboard( unsigned char key, int x, int y )
{
    // Process keyboard input.
    switch( key )
    {
       case ESC : 
          ProgramExit();
          break;
    }
}

/******************************************************************************/

BOOL GraphicsStart( void )
{
    // Start graphics window...
    if( !GRAPHICS_Start(GraphicsMode) )
    {
        printf("Cannot start GRAPHICS system.\n");
        return(FALSE);
    }

    // Set standard openGL parameters.
    GRAPHICS_OpenGL(GRAPHICS_FLAG_NONE,GraphicsBackGround);

    return(TRUE);
}

/******************************************************************************/

void GraphicsMainLoop( void )
{
    // Set various GLUT call-back functions.
    glutKeyboardFunc(KB_GLUT_KeyboardFuncInstall(GraphicsKeyboard));
    glutDisplayFunc(GraphicsDisplay);
    glutIdleFunc(GraphicsIdle);

    // Reset frequency timing objects.
    GraphicsDisplayFrequency.Reset();
    GraphicsIdleFrequency.Reset();

    // Give control to GLUT's main loop.
    glutMainLoop();
}

/******************************************************************************/

void Usage( void )
{
    printf("----------------------------------\n");
    printf("%s /C:Config(1)[,...Config(n)] /M:MetaConfig /D:DataFile\n",MODULE_NAME);
    printf("----------------------------------\n");

    exit(0);
}

/******************************************************************************/

BOOL Initialize( void )
{
int i;

    // Load the first (and possibly the only) configuration file.
    if( !ConfigLoad(ConfigFileList[0]) )
    {
        return(FALSE);
    }
    
    if( MovementType != MOVETYPE_OUTONLY )
    {
        printf("MovementType: Only OutOnly supported.\n");
        return(FALSE);
    }

    // Open list of wave files.
    if( !WAVELIST_Open(WaveList) )
    {
        printf("WAVELIST: Cannot load WAV files.\n");
        return(FALSE);
    }

    ContextFullMovementTimeData.Data(PostMoveDelayInit);

    for( i=0; (i < MISS_TRIAL_TYPES); i++ )
    {
        MissTrialsTypeTotal[i] = 0;
        MissTrialsType[i] = 0;
    }

    // Add each variable to the TrialData matrix.
    TrialData.AddVariable(VAR(ExperimentTime));
    TrialData.AddVariable(VAR(ConfigIndex));
    TrialData.AddVariable(VAR(PhaseIndex));
    TrialData.AddVariable(VAR(TrialPhase));
    TrialData.AddVariable(VAR(MovementType));
    TrialData.AddVariable(VAR(MovementDirection));
    TrialData.AddVariable(VAR(FieldIndex));
    TrialData.AddVariable(VAR(FieldType));
    TrialData.AddVariable(VAR(FieldConstants),FIELD_CONSTANTS);
    TrialData.AddVariable(VAR(FieldAngle));
    TrialData.AddVariable(VAR(PassiveWaitFirstFlag));
    TrialData.AddVariable(VAR(PassiveWaitLastFlag));
    TrialData.AddVariable(VAR(ContextType));
    TrialData.AddVariable(VAR(ContextConstants),FIELD_CONSTANTS);
    TrialData.AddVariable(VAR(TrialDelay));
    TrialData.AddVariable(VAR(InterTrialDelay));
    TrialData.AddVariable(VAR(TargetAngle));
    TrialData.AddVariable(VAR(TargetPosition));
    TrialData.AddVariable(VAR(MovementFirstDistance));
    TrialData.AddVariable(VAR(MovementSecondDistance));
    TrialData.AddVariable(VAR(ViaPosition));
    TrialData.AddVariable(VAR(SymmetryAxisAngle));
    TrialData.AddVariable(VAR(TargetResolveDistance));
    TrialData.AddVariable(VAR(MovementOrderType));
	TrialData.AddVariable(VAR(ChannelOrderType));
    TrialData.AddVariable(VAR(MovedTooFarDistance)); 
    TrialData.AddVariable(VAR(MissedViaPointDistance)); 
    TrialData.AddVariable(VAR(StartPosition));
    TrialData.AddVariable(VAR(FinishPosition));
    TrialData.AddVariable(VAR(MissTrials));
    TrialData.AddVariable(VAR(MissTrialsType),MISS_TRIAL_TYPES);
    TrialData.AddVariable(VAR(TrialDuration));
    TrialData.AddVariable(VAR(MovementReactionTime));
    TrialData.AddVariable(VAR(MovementDurationTime));
    TrialData.AddVariable(VAR(MovementFirstTime));
    TrialData.AddVariable(VAR(MovementFirstTooSlow));
    TrialData.AddVariable(VAR(MovementFirstTooFast));
    TrialData.AddVariable(VAR(MovementSecondTime));
    TrialData.AddVariable(VAR(MovementSecondTooSlow));
    TrialData.AddVariable(VAR(MovementSecondTooFast));
    TrialData.AddVariable(VAR(PMoveStartPosition));
    TrialData.AddVariable(VAR(PMoveEndPosition));
    TrialData.AddVariable(VAR(ViaToleranceTime));
    TrialData.AddVariable(VAR(ViaTimeOutTime));
    TrialData.AddVariable(VAR(PostMoveDelayTime));
    TrialData.AddVariable(VAR(PostMoveDelayInit));   
    TrialData.AddVariable(VAR(PassingViaTime));   
    TrialData.AddVariable(VAR(ViaNotMovingTime));   
    
    // Add each variable to the FrameData matrix.
    FrameData.AddVariable(VAR(TrialTime));         
    FrameData.AddVariable(VAR(State));
    FrameData.AddVariable(VAR(StateGraphics));
    FrameData.AddVariable(VAR(ForcesFunctionLatency));
    FrameData.AddVariable(VAR(ForcesFunctionPeriod));
    FrameData.AddVariable(VAR(RobotPosition));
    FrameData.AddVariable(VAR(RobotVelocity));     
    FrameData.AddVariable(VAR(RobotForces));       
    FrameData.AddVariable(VAR(HandleForces));
    FrameData.AddVariable(VAR(HandleTorques));
    FrameData.AddVariable(VAR(CursorPosition));
    FrameData.AddVariable(VAR(ForceFieldRampValue));
    FrameData.AddVariable(VAR(TargetResolveFlag));
    FrameData.AddVariable(VAR(PMoveState));
    FrameData.AddVariable(VAR(PMoveStateTime));
    FrameData.AddVariable(VAR(PMoveStateRampValue));
    FrameData.AddVariable(VAR(PMoveStatePosition));

    // Add GRAPHICS variables to FrameData matrix.
    GRAPHICS_FrameData(&FrameData);

    // Set rows of FrameData to maximum.
    FrameData.SetRows(FRAMEDATA_ROWS);

    return(TRUE);
}

/******************************************************************************/

BOOL TrialListSubset( void )
{
BOOL ok;
int i;

    TrialOffset = TotalTrials;

    // Create list of trials.
    for( ok=TRUE,TrialPhaseLast=-1,Trial=1; ((Trial <= Trials) && ok); )
    {
        for( TrialPhase=-1,i=0; (i < PHASE_MAX); i++ )
        {
            if( (Trial >= PhaseTrialRange[i][0]) && (Trial <= PhaseTrialRange[i][1]) )
            {
                TrialPhase = i;
                break;
            }
        }

        if( TrialPhase == -1 )
        {
            printf("Trial=%d Not within Phase trial ranges.\n",Trial);
            ok = FALSE;

            continue;
        }

        if( TrialPhaseLast != TrialPhase )
        {
            TrialPhaseLast = TrialPhase;
            PhaseIndex++;

            PhaseFieldIndexPermute.Init(0,PhaseFieldIndexCount[TrialPhase]-1,PhaseFieldPermute[TrialPhase]);
        }

        FieldIndex = PhaseFieldIndex[TrialPhase][PhaseFieldIndexPermute.GetNext()];
        FieldIndexTrialCount[FieldIndex]++;

        FieldType = FieldIndexType[FieldIndex];
        FieldTrials[FieldType]++;

        FieldAngle = FieldIndexAngle[FieldIndex];

        for( i=0; (i < FIELD_CONSTANTS); i++ )
        {
            FieldConstants[i] = FieldIndexConstants[FieldIndex][i];
        }

        ContextType = FieldIndexContextType[FieldIndex];

        for( i=0; (i < FIELD_CONSTANTS); i++ )
        {
            ContextConstants[i] = FieldIndexContextConstants[FieldIndex][i];
        }

        TargetAngle = ContextConstants[0];
        SymmetryAxisAngle = ContextConstants[1];
        TargetResolveDistance = ContextConstants[2];
        MovementOrderType = (int)ContextConstants[3];
        ChannelOrderType = (int)ContextConstants[4];

        switch( MovementOrderType )
        {
            case ORDER_FOLLOW_THROUGH :
                StartPosition = ViaPosition - TargetAngleVector(SymmetryAxisAngle,MovementFirstDistance);
                TargetPosition = ViaPosition + TargetAngleVector(SymmetryAxisAngle+TargetAngle,MovementSecondDistance);
                break;

            case ORDER_LEAD_IN :
                StartPosition = ViaPosition - TargetAngleVector(SymmetryAxisAngle+TargetAngle,MovementFirstDistance);
                TargetPosition = ViaPosition + TargetAngleVector(SymmetryAxisAngle,MovementSecondDistance);
                break;

            case ORDER_SINGLE_MOVEMENT :
                MovementSecondDistance = MovementFirstDistance;

                StartPosition = ViaPosition;
                TargetPosition = ViaPosition + TargetAngleVector(SymmetryAxisAngle+TargetAngle,MovementSecondDistance);
                break;
        }

        switch( MovementType )
        {
            case MOVETYPE_OUTANDBACK :
                FinishPosition = StartPosition;
                break;

            case MOVETYPE_OUTTHENBACK :
            case MOVETYPE_OUTONLY :
                FinishPosition = TargetPosition;
                break;
        }

        // Is it a single movement only to central target?
        if( !ContextFullMovementFlag[ContextType] && (MovementOrderType != ORDER_SINGLE_MOVEMENT) )
        {
            FinishPosition = ViaPosition;
        }

        // Move direction (Out,Back,OutAndBack).
        MovementDirection = MoveTypeDirection[MovementType];

        if( TrialDelayLambda != 0.0 )
        {
            // Code from RandomDotsTask for "exponential delay time".
            do
            {
                TrialDelay = TrialDelayOffset + (-1.0 * log(RandomUniform(0.00001,1.0))) / TrialDelayLambda;
            }
            while( TrialDelay > TrialDelayMax );
        }

        // Save TrialData row for this trial.
        TrialData.RowSave(TrialOffset+Trial);

        // Increment trial number depending on movement type...
        Trial += MoveTypeTrials[MovementType];
    }

    return(ok);
}

/******************************************************************************/

BOOL TrialList( void )
{
int i;
BOOL ok=TRUE;

    TotalTrials = 0;
    for( i=0; (i < FIELD_MAX); i++ )
    {
        FieldTrials[i] = 0;
    }

    // Single or multiple configuration file paradigm?
    if( ConfigFileCount == 1 )
    {
        TotalTrials = Trials;
        ConfigIndex = 0;
    }
    else
    {
        // Loop over configuration files, counting the number of trials.
        for( ok=TRUE,ConfigIndex=1; (ok && (ConfigIndex < ConfigFileCount)); ConfigIndex++ )
        {
            if( !ConfigLoad(ConfigFileList[ConfigIndex]) )
            {
                ok = FALSE;
                continue;
            }
			
            if( RestBreakHere )
            {
                RestBreakTrials[RestBreakCount] = TotalTrials;
                printf("RestBreakTrials[%d] = %d\n",RestBreakCount,TotalTrials);
                RestBreakCount++;
            }

            TotalTrials += Trials;
            printf("%d %s Trials=%d TotalTrials=%d\n",ConfigIndex,ConfigFileList[ConfigIndex],Trials,TotalTrials);
        }

        ConfigIndex = 1;
    }

    if( (TotalTrials == 0) || !ok )
    {
        return(FALSE);
    }

    // Set rows of TrialData to the number of trials.
    TrialData.SetRows(TotalTrials);

    printf("Making list of %d trials (ESCape to abort)...\n",TotalTrials);

    TotalTrials = 0;
    for( i=0; (i < FIELD_MAX); i++ )
    {
        FieldTrials[i] = 0;
    }

    // Loop over configuration files, appending each to growing trial list.
    for( ok=TRUE; (ok && (ConfigIndex < ConfigFileCount)); ConfigIndex++ )
    {
        if( ConfigIndex > 0 )
        {
            if( !ConfigLoad(ConfigFileList[ConfigIndex]) )
            {
                ok = FALSE;
                continue;
            }
        }

        // Create subset of trials for this configuration file.
        if( !TrialListSubset() )
        {
            ok = FALSE;
            continue;
        }

        TotalTrials += Trials;
        //printf("%d %s Trials=%d TotalTrials=%d\n",ConfigIndex,ConfigFileList[ConfigIndex],Trials,TotalTrials);
    }

    // Create the "back" movements here (JNI 31/May/2016).
    if( MovementType != MOVETYPE_OUTANDBACK )
    {
        // Make sure we have an odd number of trials...
        if( (TotalTrials%2) == 0 )
        {
            // Ignore the last even-numbered trial...
            TotalTrials--;
        }

        // Start at the last trial and work backwards...
        for( Trial=TotalTrials; (Trial >= 2); ) 
        {
            // Load the odd numbered trial and decrement the trial number.
            TrialData.RowLoad(Trial);
            Trial--;

            // Make changes here to turn trial into a "back" movement...
            MovementDirection = MOVEDIR_BACK;

            FieldType = FIELD_PMOVE;
            FieldTrials[FieldType]++;

            FieldAngle = 0.0;

            for( i=0; (i < FIELD_CONSTANTS); i++ )
            {
                FieldConstants[i] = 0.0;
            }	

            ContextType = PASSIVE_MOVE;

            // Set finish position of this trial to home position of the next trial.
            FinishPosition = StartPosition;
          
            // Save the even numbered "back" trial and decrement trial number.
            TrialData.RowSave(Trial);
            Trial--;
        }
    }

    for( Trial=1; (Trial <= (TotalTrials-2)); Trial+=2 )
    {
        TrialData.RowLoad(Trial);
        ContextTypeLast = ContextType;
        TrialData.RowLoad(Trial+2);
        ContextTypeNext = ContextType;

        if( (ContextTypeLast != PASSIVE_WAIT) && (ContextType == PASSIVE_WAIT) )
        {
            TrialData.RowLoad(Trial+2);
            PassiveWaitFirstFlag = TRUE;
            TrialData.RowSave(Trial+2);
        }

        if( (ContextTypeLast == PASSIVE_WAIT) && (ContextType != PASSIVE_WAIT) )
        {
            TrialData.RowLoad(Trial);
            PassiveWaitLastFlag = TRUE;
            TrialData.RowSave(Trial);
        }

        TrialData.RowLoad(Trial+1);

        if( (ContextTypeLast == PASSIVE_WAIT) && (ContextTypeNext == PASSIVE_WAIT) && (FieldType == FIELD_PMOVE) && !PassiveWaitLastFlag )
        {
            FieldType = FIELD_SAMEASLAST;
            TrialData.RowSave(Trial+1);
        }
    }

    if( !ok )
    {
        return(FALSE);
    }

    printf("RestBreakCount = %d\n",RestBreakCount);
    for( i=0; (i < RestBreakCount); i++ )
    {
        printf("RestBreakTrials[%d] = %d\n",i,RestBreakTrials[i]);
    }

    // Total number of trails.
    Trials = TotalTrials;

    // Save trial list to file.
    ok = DATAFILE_Save(TrialListFile,TrialData);
    printf("%s %s Trials=%d.\n",TrialListFile,STR_OkFailed(ok),TrialData.GetRows());

    // Reset trial number, etc.
    Trial = 1;
    TrialSetup();

    StateNext(STATE_INITIALIZE);

    return(TRUE);
}

/******************************************************************************/

void main( int argc, char *argv[] )
{
    // Initialize MOTOR.LIB, command-line parameters, configuration files, etc.
    if( !MOTOR_Parameters(argc,argv,DataName,DataFile,TrialListFile,ConfigFileCount,ConfigFileList) )
    {
        exit(0);
    }

    // Initialize variables, etc.
    if( !Initialize() )
    {
        ProgramExit();
    }

    // Create trial list.
    if( !TrialList() )
    {
        printf("TrialList() Failed.\n");
        ProgramExit();
    }

    // Start the robot.
    if( DeviceStart() )
    {
        // Start the graphics system.
        if( GraphicsStart() )
        {
            // The experiment is run as part of graphics processing.
            GraphicsMainLoop();
        }
    }

    // Exit the program.
    ProgramExit();
}

/******************************************************************************/

