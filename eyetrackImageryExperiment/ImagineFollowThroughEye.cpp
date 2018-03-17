/******************************************************************************/
/*                                                                            */
/* MODULE  : ImagineFollowThroughEye.cpp                                      */
/*                                                                            */
/* PURPOSE : Follow through force-field adaptation paradigm with imagery.     */
/*                                                                            */
/* DATE    : 01/Oct/2015                                                      */
/*                                                                            */
/* CHANGES                                                                    */
/*                                                                            */
/* V1.0  JNI 01/Oct/2015 - Initial development (DynamicContext.cpp)           */
/*                                                                            */
/* V1.1  JNI 13/Oct/2015 - Ongoing development.                               */
/*                                                                            */
/* V1.2  HRS 01/Dec/2015 - Development of DynamicFollowThrough.cpp            */
/*                                                                            */
/* V1.3  HRS 20/Nov/2017 - Develop to ImagineFollowThrough.cpp, added button. */
/*                                                                            */
/* V1.4  JNI 22/Nov/2017 - Added eye-tracker code for fixation control.       */
/*                                                                            */
/* V1.5  JNI 29/Nov/2017 - Save miss trials (for fixation control).           */
/*                                                                            */
/******************************************************************************/

#define MODULE_NAME "ImagineFollowThroughEye"

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
BOOL    RobotActiveFlag=FALSE;
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
int     MovementType=MOVETYPE_OUTANDBACK;
int     MovementDirection=MOVEDIR_OUTBACK;

matrix  TextPosition(3,1);
matrix  HomePosition(3,1);
matrix  ViaPosition(3,1);
double  HomeTolerance=0.50;
double  HomeToleranceTime=0.1;
double  ViaTolerance=0.50;
double  ViaToleranceTime=0.05;
double  ViaTimeOutTime=0.15;
double  ViaSpeedThreshold=20;
double  FollowSpeedQuickTarget=45;
double  FollowSpeedSlowTarget=25;
double  FollowSpeedTarget=45;
double  FollowSpeedTolerance=5;
double  MovementReactionTimeOut=0.5;
double  MovementDurationTimeOut=0.8;
double  MovementDurationTooFast=0.2;
double  MovementDurationTooSlow=0.4;
double  MovementDurationTooFastToVia=0.2;
double  MovementDurationTooSlowToVia=0.35;

double  FixateSpatialTolerance=3.0;    // cm
double  FixateTemporalTolerance=0.0;   // sec
matrix  FixateCrossPosition(3,1);
double  FixateCrossWidth=2.0;          // OpenGL units?
double  FixateCrossSize=0.5;           // cm
BOOL    FixateRequiredFlag=TRUE;
BOOL    FixateFlag=FALSE;

double  ErrorWait=0.5;
double  TrialDelay=0.3;
double  InterTrialDelay=0.5;
double  FeedbackTime=0.5;
double  NotMovingSpeed=1; // cm/sec
double  NotMovingTime=0.1;
TIMER   NotMovingTimer("NotMoving");
double  TargetSpeedVia=50; // cm/sec
double  TargetSpeedTarget=40;  // cm/sec
double  TargetSpeedTolerance=10;

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

#define TARGETS 32
double  TargetAngles[TARGETS];
//int     TargetCount;
BOOL    TargetTestFlag=FALSE;

matrix  CursorPosition(3,1);
int     CursorColor=RED;           
STRING  CursorColorText="RED";
double  CursorRadius=0.5;

matrix  ForceFieldForces(3,1);
BOOL    ForceFieldStarted=FALSE;
matrix  ForceFieldPosition(3,1);
RAMPER  ForceFieldRamp;
double  ForceFieldRampTime=0.1;

matrix  WallForces(3,1);
BOOL    WallStarted=FALSE;
double  WallDistance;
//matrix  WallPosition(3,1);
RAMPER  WallRamp;
double  WallRampTime=0.1;
double  SpringConstant=-60;  // N/cm
double  DampingConstant=0.05;

int ButtonPress=0;

BOOL	MovedTooFar=FALSE;
double	MaxSpeed=0.0;

int     HomeColor=WHITE;
int     NotHomeColor=GREY;
double  HomeRadius=0.5;
double  HomeWidth=3.0;

int     TargetColor=YELLOW;
STRING  TargetColorText="YELLOW";
int     TargetHitColor=YELLOW;
STRING  TargetHitColorText="YELLOW";


double  TargetRadius=0.5;
double  TargetWidth=3.0;

int     ViaColor=GREY;
int     ImagineViaColor=TURQUOISE;
int		ViaPassedColor=GREY;

double  ViaRadius=0.5;

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

TIMER  MovementDurationTimer("MovementDuration");
TIMER  MovementDurationToViaTimer("MovementDuration");
TIMER  MovementReactionTimer("MovementReaction");
TIMER  MovementFinishedTimer("MovementFinished");
TIMER  PassingViaTimer("PassingVia");

TIMER  TrialTimer("Trial");
TIMER  InterTrialDelayTimer("InterTrialDelay");
double TrialTime;
double TrialDuration=0.0;
int    Trial;
int    TrialNumber=0;
BOOL   TrialRunning=FALSE;
BOOL   PassedVisibleDistance=FALSE;

// Field types.
#define FIELD_NONE      0
#define FIELD_VISCOUS   1
#define FIELD_CHANNEL   2
#define FIELD_PMOVE     3
#define FIELD_MAX       4

// Follow through target types.
#define TARGET_STATIC_ON	0
#define TARGET_APPEAR		1
#define TARGET_STOP		2
#define TARGET_STOP_WARNING	3
#define TARGET_STATIC_OFF	4

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
matrix  FieldHomePosition[FIELD_INDEX];


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
double FieldConstants[FIELD_CONSTANTS];
double FieldAngle;
int    ContextType;
double ContextConstants[FIELD_CONSTANTS];
matrix TargetPosition(3,1);
int    TargetIndex;
double TargetAngle;
double TargetDistance;
double HomeAngle;
BOOL   MissTrialFlag=FALSE;
int    MissTrials=0;
int    MissTrialsFixation=0;
int    MissTrialsTotal=0;
int    MissTrialsFixationTotal=0;
double MissTrialsPercent=0.0;
double MovementReactionTime=0.0;
double MovementDurationTime=0.0;
double MovementDurationToViaTime=0.0;
matrix StartPosition(3,1);
matrix FinishPosition(3,1);
matrix FinishPositionTemp(3,1);

int    RobotFieldType;
double RobotFieldConstants[FIELD_CONSTANTS];
double RobotFieldAngle;
matrix RobotFieldMatrix;

TIMER   ExperimentTimer("Experiment");
double  ExperimentTime;
double  ExperimentSeconds;
double  ExperimentMinutes;

/******************************************************************************/

// Eye tracker variables. (1)
STRING EyeTrackerConfig="";
BOOL   EyeTrackerFlag=FALSE;
int    EyeTrackerFrameCount=0;
BOOL   EyeTrackerFrameReady=FALSE;
double EyeTrackerTimeStamp=0.0;
double EyeTrackerEyeXY[2]={ 0.0,0.0 };
matrix EyeTrackerEye(3,1);
double EyeTrackerPupilSize=0.0;
TIMER_Frequency EyeTrackerFrameFrequency("EyeTrackerFrameFrequency");

/******************************************************************************/
#define STATE_INITIALIZE   0
#define STATE_SETUP        1
#define STATE_HOME         2
#define STATE_START        3
#define STATE_DELAY        4
#define STATE_GO           5
#define STATE_MOVEWAIT     6
#define STATE_MOVING0      7
#define STATE_MOVING1      8
#define STATE_FEEDBACK	   9
#define STATE_FINISH      10
#define STATE_NEXT        11
#define STATE_INTERTRIAL  12
#define STATE_EXIT        13
#define STATE_TIMEOUT     14
#define STATE_ERROR       15
#define STATE_REST        16
#define STATE_EYETRACKER  17 // Eye tracker calibration state. (2)
#define STATE_MAX         18

int   State=STATE_INITIALIZE;
int   StateLast;
BOOL  StateFirstFlag=FALSE;
int   StateGraphics=STATE_INITIALIZE;
int   StateGraphicsLast;
char *StateText[] = { "Initialize","Setup","Home","Start","Delay","Go","MoveWait","Moving0","Moving1","Feedback","Finish","Next","InterTrial","Exit","TimeOut","Error","Rest","EyeTracker" };
BOOL  StateLoopTask[STATE_MAX] = { FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,TRUE,TRUE,TRUE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE };
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
    CONFIG_set(VAR(EyeTrackerConfig)); // Eye tracker configuration file. (3)
    CONFIG_setBOOL(VAR(FixateRequiredFlag));
    CONFIG_set(VAR(TextPosition));
    CONFIG_set("CursorColor",CursorColorText);
    CONFIG_set(VAR(CursorRadius));
    CONFIG_set(VAR(TargetDistance));
    CONFIG_set("TargetColor",TargetColorText);
    CONFIG_set(VAR(TargetRadius));
    CONFIG_set(VAR(TargetWidth));
    CONFIG_set(VAR(ViaRadius));
    CONFIG_set(VAR(ViaPosition));
    CONFIG_set(VAR(WallDistance));
    CONFIG_set(VAR(HomeRadius));
    CONFIG_set(VAR(HomeWidth));
    CONFIG_set(VAR(HomeTolerance));
    CONFIG_set(VAR(HomeToleranceTime));
    CONFIG_set(VAR(ViaTolerance));
    CONFIG_set(VAR(ViaToleranceTime));
    CONFIG_set(VAR(ViaTimeOutTime));
    CONFIG_set(VAR(ViaSpeedThreshold));
    CONFIG_set(VAR(FollowSpeedQuickTarget));
    CONFIG_set(VAR(FollowSpeedSlowTarget));
    CONFIG_set(VAR(FollowSpeedTolerance));
    CONFIG_set(VAR(PostMoveDelayInit));

    CONFIG_set("MovementType",MovementTypeString);
    CONFIG_set(VAR(MovementReactionTimeOut));
    CONFIG_set(VAR(MovementDurationTimeOut));
    CONFIG_set(VAR(MovementDurationTooFast));
    CONFIG_set(VAR(MovementDurationTooSlow)); 
    CONFIG_set(VAR(MovementDurationTooFastToVia)); 
    CONFIG_set(VAR(MovementDurationTooSlowToVia)); 
    CONFIG_set(VAR(ErrorWait));
    CONFIG_set(VAR(TrialDelay));
    CONFIG_set(VAR(InterTrialDelay));
    CONFIG_set(VAR(FeedbackTime));
    CONFIG_set(VAR(NotMovingSpeed));
    CONFIG_set(VAR(NotMovingTime));
    CONFIG_set(VAR(ForceFieldRampTime));
    CONFIG_set(VAR(TargetSpeedVia));
    CONFIG_set(VAR(TargetSpeedTarget));
    CONFIG_set(VAR(TargetSpeedTolerance));

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
        CONFIG_set("HomePosition",FieldHomePosition[i]);
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
            
            FieldHomePosition[i].dim(3,1);
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

    // Eye tracker in use? (4)
    EyeTrackerFlag = !STR_null(EyeTrackerConfig);

    // Count the rest-breaks in case they have been specified.
    /*
	for( RestBreakCount=0; ((RestBreakCount < RESTBREAK_MAX) && (RestBreakTrials[RestBreakCount] != 0)); RestBreakCount++ );

    printf("ConfigLoad(%s) Load %s.\n",file,STR_OkFailed(ok));
    CONFIG_list(printf);
	*/
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
        //P(2,1) -= 3.0;
        //P(2,1) = 9.0;
        
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
    ForceFieldPosition = RobotPosition;
    ForceFieldStarted = TRUE;
    ForceFieldRamp.Up();
	
    if( FieldType == FIELD_PMOVE )
    {
        ForceFieldRamp.One();
        RobotPMoveStart();
    }
}


/******************************************************************************/

void ForceFieldStop( void )
{
	if( ForceFieldStarted )
	{
		ForceFieldRamp.Down();
		ForceFieldStarted = FALSE;
	}
}


/******************************************************************************/

void WallStart( void )
{
    WallStarted = TRUE;
    WallRamp.Up();
}

/******************************************************************************/

void WallStop( void )
{
		WallRamp.Down();
}

/******************************************************************************/

void RobotForcesFunction( matrix &position, matrix &velocity, matrix &forces )
{
int i,j;
static matrix P,V,R,_R;
static matrix P1,V1,R1,_R1;
static double HomeDistance, WallYPosition;
static BOOL ok;
BOOL BarrierOn=FALSE;

    // Monitor timing of Forces Function (values saved to FrameData).
    ForcesFunctionPeriod = RobotForcesFunctionFrequency.Loop();
    RobotForcesFunctionLatency.Before();

    TrialTime = TrialTimer.ElapsedSeconds();

    // Kinematic data passed from robot API.
    RobotPosition = position;
    RobotVelocity = velocity;
    RobotSpeed = norm(RobotVelocity);

    RobotActiveFlag = ROBOT_Activated(RobotID);

    // Zero forces.
    ForceFieldForces.zeros();
    RobotForces.zeros();

    // Read raw sensor values from Sensoray card.
    ROBOT_SensorRead(RobotID);

    // Get Force/Torque sensor if required.
    if( RobotFT && ROBOT_SensorOpened_DAQFT(RobotID) )
    {
        ROBOT_Sensor_DAQFT(RobotID,HandleForces,HandleTorques);
    }

    P = RobotPosition;
    CursorPosition = RobotPosition;

    // Process force-field type.
    switch( ForceFieldStarted ? RobotFieldType : FIELD_NONE )
    {
        case FIELD_NONE :
           break;

        case FIELD_VISCOUS :   // Viscous force field.

           ForceFieldForces = RobotFieldConstants[0] * RobotFieldMatrix * RobotVelocity;
           break;

        case FIELD_CHANNEL :
           // Next rotate position along the channel between the home position and the via point.

           SPMX_romxZ(D2R(HomeAngle),R);
           SPMX_romxZ(D2R(-HomeAngle),_R);

           P = R * (RobotPosition - ForceFieldPosition);
           V = R * RobotVelocity;

           // Calculate perpendicular (X) channel forces.
           ForceFieldForces(1,1) = (RobotFieldConstants[0] * P(1,1)) + (RobotFieldConstants[1] * V(1,1));

           // Rotate back to original.
           ForceFieldForces = _R * ForceFieldForces;
           break;

        case FIELD_PMOVE :
           RobotPMoveUpdate(ForceFieldForces);
           break;

    }

	if (FieldType != FIELD_PMOVE)
	{
       // HRS: Wall barrier no longer in use.
	  // Process wall barrier
		/*
		BarrierOn = ContextConstants[3];
		
		SPMX_romxZ(D2R(HomeAngle),R1);
		SPMX_romxZ(D2R(-HomeAngle),_R1);
           	P1 = R1 * (RobotPosition - ViaPosition);
	
		if( P1(2,1) >= WallDistance )
		{
			MovedTooFar=TRUE;
		}
		
	   */
	
	    BarrierOn = ContextConstants[3];
		if (BarrierOn)
		{	
			WallForces.zeros();
						
			SPMX_romxZ(D2R(HomeAngle),R1);
			SPMX_romxZ(D2R(-HomeAngle),_R1);
            P1 = R1 * (RobotPosition - ViaPosition);
			V1 = R1 * RobotVelocity;	
			// HRS: Wall barrier no longer in use.
            //HomeDistance 	= sqrt( pow(HomePosition(1,1),2) + pow(HomePosition(2,1),2) );
			//WallYPosition   = WallDistance - HomeDistance;
			

			if( P1(2,1) >= WallDistance )
			{
				WallForces(2,1) = SpringConstant * (P1(2,1) - WallDistance) + DampingConstant * V1(2,1);
				WallForces = _R1 * WallForces;  // Rotate back to the original
				MovedTooFar=TRUE;
							
			}
		}
		else
		{
			SPMX_romxZ(D2R(HomeAngle),R1);
			SPMX_romxZ(D2R(-HomeAngle),_R1);
           	P1 = R1 * (RobotPosition - ViaPosition);
		
			if( P1(2,1) >= WallDistance )
			{
				MovedTooFar=TRUE;
			}
		}
	
	}


    // Process Finite State Machine.
    StateProcessLoopTask();

    // Monitor timing of Forces Function (values saved to FrameData).
    ForcesFunctionLatency = RobotForcesFunctionLatency.After();

	RobotForces = (ForceFieldRamp.RampCurrent() * ForceFieldForces) + WallForces;
	//RobotForces = (ForceFieldRamp.RampCurrent() * ForceFieldForces) + (WallRamp.RampCurrent() * WallForces); // HRS: Wall barrier no longer in use.

    // Get next frame of eye tracker data. (5)
    if( EyeTrackerFlag )
    {
        ok = EYET_FrameNext(EyeTrackerTimeStamp,EyeTrackerEyeXY,EyeTrackerPupilSize,EyeTrackerFrameReady);

        if( EyeTrackerFrameReady )
        {
            EyeTrackerEye(1,1) = EyeTrackerEyeXY[0];
            EyeTrackerEye(2,1) = EyeTrackerEyeXY[1];

            EyeTrackerFrameCount++;
            EyeTrackerFrameFrequency.Loop();
        }
    }

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
    ForceFieldRamp.Close();

    WallRamp.Stop();
    WallRamp.Close();

    RobotID = ROBOT_INVALID;

    // Stop eye tracker if required. (6)
    if( EyeTrackerFlag )
    {
        EYET_Close();
        EyeTrackerFrameFrequency.Results();
    }
}

/******************************************************************************/

BOOL DeviceStart( void )
{
BOOL ok=TRUE;

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

    if( ok )
    {
        if( !ForceFieldRamp.Start(ForceFieldRampTime) )
        {
            printf("ForceFieldRamp: Start failed.\n");
            ok = FALSE;
        }
    }

    if( ok )
    {
        if( !WallRamp.Start(WallRampTime) )
        {
            printf("WallRamp: Start failed.\n");
            ok = FALSE;
        }
    }

    // Start eye tracker if required. (7)
    if( ok && EyeTrackerFlag )
    {
        // The GraphicsText function sets a string to be displayed by OpenGL. 
        ok = EYET_Open(EyeTrackerConfig,GraphicsText);
        printf("EYET_Open(%s) %s.\n",EyeTrackerConfig,STR_OkFailed(ok));
        EyeTrackerFrameFrequency.Reset();
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

    flag = RobotHome(StartPosition,HomeTolerance);

    return(flag);
}

/******************************************************************************/

BOOL MovementStarted( void )
{
BOOL flag;

    flag = !RobotHome(StartPosition,HomeTolerance);

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
    if( !RobotHome(FinishPosition,HomeTolerance) )
    {
		MovementFinishedTimer.Reset();    
    }
     
    // Has the robot been in the finish position for the required amount of time?
    if ( (ContextType == TARGET_STOP_WARNING) || (ContextType == TARGET_STOP) )
	{
            if( ButtonPress )
            {
                flag = TRUE;
            }
		/*PostMoveDelayTime = ContextFullMovementTimeData.Mean() - MovementDurationToViaTime + ViaToleranceTime;
		if( PassingViaTimer.ExpiredSeconds(PostMoveDelayTime) ) 
		{
			flag = TRUE;
		}*/
	}
	else
	{
		if( MovementFinishedTimer.ExpiredSeconds(HomeToleranceTime) )
		{
			flag = TRUE;
		}
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
    MissTrialFlag = FALSE;

    // Set robot force field variables.
    RobotFieldType = FieldType;

    for( i=0; (i < FIELD_CONSTANTS); i++ )
    {
        RobotFieldConstants[i] = FieldConstants[i];
    }

    RobotFieldAngle = FieldAngle;
    SPMX_romxZ(D2R(RobotFieldAngle),RobotFieldMatrix);

    TrialRunning = FALSE;
	PassedVisibleDistance = FALSE;
	MovedTooFar = FALSE;
	StateGraphicsNext(State);

    printf("TrialSetup: Trial=%d FieldType=%d ContextType=%d\n",Trial,FieldType,ContextType);
}

/******************************************************************************/

void TrialStart( void )
{
    printf("Starting Trial %d...\n",Trial);
    printf("TargetAngle=%.1lf(deg) Phase=%d Field=%d FieldConstant=%.2lf,%.2lf\n",TargetAngle,TrialPhase,FieldType,FieldConstants[0],FieldConstants[1]);
	disp(TargetPosition);
	disp(FinishPosition);

    TrialTimer.Reset();
    TrialTime = TrialTimer.ElapsedSeconds();
    TrialRunning = TRUE;
    MovedTooFar = FALSE;
    // Start force field.
    ForceFieldStart();

    // Start recording frame data for trial.
    FrameStart();
}

/******************************************************************************/

BOOL TrialSave( void )
{
BOOL ok=FALSE;

    ExperimentTime = ExperimentTimer.ElapsedSeconds();
    MissTrials = MissTrialsTotal;
    MissTrialsFixation = MissTrialsFixationTotal;
    TrialNumber = Trial; // For saving miss trials.

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

BOOL TrialStop( BOOL AbortFlag )
{
BOOL ok;

    TrialRunning = FALSE;

    if( AbortFlag )
    {
        printf("Aborting Trial %d...\n",Trial);
    }
    else
    {
        printf("Stopping Trial %d...\n",Trial);
    }

    // Stop recording frame for trial.
    FrameStop();

    // Stop force field.
    ForceFieldStop();

    // Stop wall field.
    WallStop();

    TrialDuration = TrialTimer.ElapsedSeconds();
    InterTrialDelayTimer.Reset();

    // Save the data for this trial (changed for saving miss trials).
    ok = TrialSave();

    return(ok);
}

/******************************************************************************/

BOOL TrialStop( void )
{
BOOL ok;

    ok = TrialStop(FALSE); // FALSE = Stop the trial.

    return(ok);
}

/******************************************************************************/

BOOL TrialAbort( void )
{
BOOL ok;

    ok = TrialStop(TRUE); // TRUE = Abort the trial.

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
    MessageSet("Frame data full",LIGHTBLUE);
    printf("Error: Frame data full\n");
}

/******************************************************************************/

void ErrorMessage( char *str )
{
    MessageSet(str,LIGHTBLUE);
    printf("Error: %s\n",str);
}

/******************************************************************************/

void ErrorMoveWaitTimeOut( void )
{
    MessageSet("Move After Beep",LIGHTBLUE);
    printf("Error: MoveWaitTimeOut\n");
}

/******************************************************************************/

void ErrorFixateCross( void )
{
    MessageSet("Fixate Cross");
    printf("Error: FixateCross\n");
}

/******************************************************************************/

void ErrorMoveTooSlow( void )
{
    MessageSet("Too Slow");
    printf("Error: TooSlow\n");
}

/******************************************************************************/

void FeedbackMessage( void )
{
static BOOL DualFeedback;
DualFeedback = ContextConstants[5];

    if( !FixateFlag )
    {
        GraphicsText("Fixate Cross");  
        return;
    }

//	Duration criteria for feedback, and positive feedback
	if ( (ContextType == TARGET_STOP_WARNING) || (ContextType == TARGET_STOP) )
	{
			if (MovedTooFar)
			{
				GraphicsText("Moved Too Far");  
			}		
	}
	else
	{
        	if( MovementDurationTime >= MovementDurationTooSlow )
           	{
               	    GraphicsText("Too Slow");                
          	}
			else 
			{
				if (MovementDurationTime < MovementDurationTooFast )
				{
		    		   GraphicsText("Too Fast");                
				}
				else
				{
		    		   GraphicsText("Correct Speed");
				}
			}
	}
}

/******************************************************************************/

void ErrorMoveTimeOut( void )
{
    MessageSet("Too Slow",LIGHTBLUE);
    printf("Error: MoveTimeOut\n");
}

/******************************************************************************/

void ErrorMoveMissedVia( void )
{
    MessageSet("Missed Via Target",LIGHTBLUE);
    printf("Error: MissedVia\n");
}

/******************************************************************************/

void ErrorMovePassedVia( void )
{
    MessageSet("Passed Target",LIGHTBLUE);
    printf("Error: Didn'tStopAtVia\n");
}

/******************************************************************************/

void ErrorMoveTooSoon( void )
{
    MessageSet("Moved Too Soon",LIGHTBLUE);
    printf("Error: MoveTooSoon\n");
}

/******************************************************************************/

void ErrorViaTimeout( void )
{
    MessageSet("Via Target Timeout",LIGHTBLUE);
    printf("Error: ViaTimeout\n");
}
/******************************************************************************/

void ErrorRobotInactive( void )
{
    MessageSet("Handle switch",LIGHTBLUE);
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

void MissTrial( BOOL fixation )
{
    MissTrialFlag = TRUE;
    MissTrialsTotal++;

    if( fixation )
    {
        MissTrialsFixationTotal++;
    }

    MissTrialsPercent = 100.0 * ((double)MissTrialsTotal / (double)Trial);
    printf("\nMiss Trials = %d/%d (%.0lf%%)\n\n",MissTrialsTotal,Trial,MissTrialsPercent);

    ErrorState(STATE_SETUP);
}

/******************************************************************************/

void MissTrial( void )
{
    MissTrial(FALSE);
}

/******************************************************************************/

void CheckGaze( void )
{
static matrix D;
static double d;

    if( !EyeTrackerFlag || !FixateRequiredFlag )
    {
        FixateFlag = TRUE;
        return;
    }

    EyeTrackerEye(3,1) = FixateCrossPosition(3,1);
    D = EyeTrackerEye - FixateCrossPosition;
    d = norm(D);

    FixateFlag = (d <= FixateSpatialTolerance);

    if( (State >= STATE_START) && (State < STATE_FEEDBACK) && (FieldType != FIELD_PMOVE) && !FixateFlag )
    {
        ErrorFixateCross();
        //TrialAbort(); // For saving miss trials (V1.5)
        MissTrial(TRUE);
    }
}

/******************************************************************************/

void TrackSpeed( void )
{
	//Monitor the speed to extract the maximum from the trial
	if (RobotSpeed > MaxSpeed)
	{
		MaxSpeed = RobotSpeed;
	}
}

/******************************************************************************/

BOOL PassingVia( void )
{
    BOOL flag=FALSE;
    
	// Is robot passing through the via position?
    if( !RobotHome(ViaPosition,ViaTolerance) )
    {
        PassingViaTimer.Reset();		
    }
	else
	{
		// Ramp down the trial forces
		ForceFieldStop();		
	}
 
// Has the robot been in the via position for the required amount of time?
    if( PassingViaTimer.ExpiredSeconds(ViaToleranceTime) )
    {
		// Has the robot velocity dropped below a threshold?
		if (RobotSpeed <= ViaSpeedThreshold)
		{
	 		flag = TRUE;
    	}
	}
    return(flag);
}

/******************************************************************************/
void HitWall( void )
{
	BOOL BarrierOn = ContextConstants[3];

    // HRS: Wall barrier no longer in use.
    /*
	//Display warning if moved too far past via point
	if ( BarrierOn && ( ContextType == TARGET_STOP_WARNING ) )
	{
		if ( MovedTooFar )
		{
			ErrorMovePassedVia();
	       		MovedTooFar=FALSE;
			if ( FieldType != FIELD_PMOVE)
			{
				TrialAbort(); // Abort the current trial.
				MissTrial();  // Generate miss trial.
			}
			

		}
	}
    */

	if ( MovedTooFar )
	{
		ErrorMoveMissedVia();
       		MovedTooFar=FALSE;
		if ( FieldType != FIELD_PMOVE)
		{
                        //TrialAbort(); // For saving miss trials (V1.5)
			MissTrial();  // Generate miss trial.
		}
	}
}


/******************************************************************************/

void StateProcessLoopTask( void )
{
    // Only some states are processing in the LoopTask.
    if( !StateLoopTask[State] )
    {
        return;
    }

    // State processing.
    switch( State )
    {
        case STATE_MOVEWAIT :
           if( MovementStarted() || (FieldType == FIELD_PMOVE) )
           {
               MovementDurationTimer.Reset();
               MovementDurationToViaTimer.Reset();
               MovementReactionTime = MovementReactionTimer.ElapsedSeconds();
			  
			   MovedTooFar=FALSE;
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
					StateNext(STATE_FEEDBACK);
					break;
				}
				break;
			}
					
			//Display warning if moved too far past via point
			HitWall();
			
			if( PassingVia()  )
           {
			   MovementDurationToViaTime = MovementDurationToViaTimer.ElapsedSeconds();

               ButtonPress = 0;
               StateNext(STATE_MOVING1);
               break;
		   }
        
		   if( MovementDurationTimer.ExpiredSeconds(MovementDurationTimeOut) && (FieldType != FIELD_PMOVE) )
           {
               StateNext(STATE_TIMEOUT);
               break;
           }

           break;


	   case STATE_MOVING1 :
		
            // Monitor the speed to extract max
            TrackSpeed();
            
            if( MovementFinished()  )
            {
				MovementDurationTime = MovementDurationTimer.ElapsedSeconds();
				if (!( (ContextType == TARGET_STOP_WARNING) || (ContextType == TARGET_STOP) ))
				{   
					ContextFullMovementTimeData.Data(MovementDurationTime); // store this for full movements only
				}
				StateNext(STATE_FEEDBACK);
                break;
            }
           
			if( FieldType == FIELD_PMOVE )
           {
               break;
           }

            if( MovementDurationTimer.ExpiredSeconds(MovementDurationTimeOut) )
            {
                StateNext(STATE_TIMEOUT);
                break;
            }
			break;

    }
}

/******************************************************************************/

void StateProcess( void )
{
double d;

    // Check that robot is in a safe state.
    if( !ROBOT_Safe(ROBOT_ID) )
    {
        printf("Robot not safe.\n");
        ProgramExit();
    }

    // Eye tracker finite state machine processing. (8)
    EYET_StateProcess();

    // Special processing while a trial is running.
    if( (State >= STATE_START) && (State <= STATE_MOVING1) )
    {
        if( !RobotActive() )
        {
            // If robot is not active, abort current trial.
            ErrorRobotInactive();
            //TrialAbort(); // For saving miss trials (V1.5)
            MissTrial();
        }
        else
        if( FrameData.Full() )
        {
            // Abort current trial if frame data is full.
            ErrorFrameDataFull();
            //TrialAbort(); // For saving miss trials (V1.5)
            MissTrial();
        }
    }

    CheckGaze();

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
           if( TargetTestFlag )
           {
               break;
           }

           ExperimentTimer.Reset();

           // Eye tracker calibration if required. (9)
           if( EyeTrackerFlag )
           {
               EYET_CalibrateStart(TRUE); // TRUE = test calibration afterwards.
               StateNext(STATE_EYETRACKER);
               break;
           }

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
           else
           {
               // Reset the max speed recording here so the pmove doesn't overwrite it	
               MaxSpeed = 0.0;
           }

           StateNext(STATE_HOME);
           break;

        case STATE_HOME :
           // Start trial when robot in home position (and stationary and active).
			WallForces.zeros();
           if (FieldType == FIELD_PMOVE)
           {
               StateNext(STATE_START);
               break;
           }

	   if( RobotNotMoving() && RobotHome() && RobotActive() && FixateFlag )
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
           if( StateTimer.ExpiredSeconds(TrialDelay) )
           {
               StateNext(STATE_GO);
               break;
           }

           if( MovementStarted() )
           {
               ErrorMoveTooSoon();
               //TrialAbort(); // For saving miss trials (V1.5)
               MissTrial();
           }
           break;

        case STATE_GO :
           // Go signal to cue movement.
           MovementReactionTimer.Reset();
           BeepGo();
           StateNext(STATE_MOVEWAIT);
           break;

        case STATE_MOVEWAIT :
           // Process in the robot forces function (LoopTask)
           break;

        case STATE_MOVING0 :
           // Process in the robot forces function (LoopTask)
           break;

        case STATE_MOVING1 :
            // Process in the robot forces function (LoopTask)
            break;

       case STATE_FEEDBACK :
	   if( FieldType == FIELD_PMOVE )
	   {		
               	StateNext(STATE_FINISH);
                break;
           }

           // Display feedback on movement speeds to subject
           FeedbackMessage();
  	   
       	   if( StateTimer.ExpiredSeconds(FeedbackTime) )
       	   {
               StateNext(STATE_FINISH);
       	   }
           break;

        case STATE_FINISH :
           // Stop and save trial.
           if( !TrialStop() ) // Also saves the trial.
           {
               printf("Cannot stop / save Trial %d.\n",Trial);
               StateNext(STATE_EXIT);
               break;
           }

	   if( RestBreakNow() )
           {
	       MessageClear();
               StateNext(STATE_REST);
               break;
           }

           StateNext(STATE_NEXT);
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
           //HitWall();

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
                    ErrorMoveMissedVia();  
				   break;

               case STATE_MOVING1 :
                   ErrorMoveTimeOut();
                   break;
                   
               default :
                  ErrorMessage(STR_stringf("%s TimeOut",StateText[StateLast]));
                  break;
           }

           //TrialAbort(); // For saving miss trials (V1.5)
           MissTrial();  // Generate miss trial.
           break;

        case STATE_ERROR :
           if( !StateTimer.ExpiredSeconds(ErrorWait) )
           {
               break;
           }

           // Do trial abort here now, for saving miss trials.
           if( !TrialAbort() )
           {
               printf("Cannot abort / save Trial %d.\n",Trial);
               StateNext(STATE_EXIT);
               break;
           }

           ErrorResume();
           break;

        case STATE_REST :
           RestBreakRemainSeconds = (RestBreakSeconds - StateTimer.ElapsedSeconds());
           RestBreakRemainPercent = (RestBreakRemainSeconds / RestBreakSeconds);

           if( RestBreakRemainSeconds > 0.0 )
           {
               break;
           }

           // Eye tracker calibration if required. (10)
           if( EyeTrackerFlag )
           {
               EYET_CalibrateStart(TRUE);
               StateNext(STATE_EYETRACKER);
               break;
           }

           StateNext(STATE_NEXT);
           break;


        case STATE_EYETRACKER :
           // Eye tracker state, stay here until it becomes idle. (11)
           if( EYET_StateIdle() )
           {
               StateNext(STATE_SETUP);
               break;
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

matrix TargetAngleVector( double angle )
{
static matrix vector(3,1);

   // Create a unit vector for target angle.
   vector(1,1) = sin(D2R(angle));
   vector(2,1) = cos(D2R(angle));
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

void GraphicsDisplayTarget( void )
{
static matrix posn(3,1);
static double FollowAngle;
static int    FollowCueColor;
static double HomeDistance, VisibleDistance;
static matrix P,R;
static double Det, Dot;
static matrix P1(3,1), P2(3,1), V1(3,1), V2(3,1), H(3,1);
BOOL   BarrierOn;
BOOL   ColourCuedFaster;
static BOOL   DualFeedback;
int attr;

	FollowAngle	  = ContextConstants[0];
	FollowCueColor  = GREEN;
	HomeDistance 	  = sqrt( pow(HomePosition(1,1),2) + pow(HomePosition(2,1),2) );
	VisibleDistance   = ContextConstants[2] - HomeDistance;
	ColourCuedFaster = ContextConstants[4];
	DualFeedback = ContextConstants[5];

	//printf("\nHomeDistance=%.1f  VisibleDistance=%.1f", HomeDistance, VisibleDistance);

	BarrierOn	  = ContextConstants[3];

	SPMX_romxZ(D2R(HomeAngle),R);
  	P = R * CursorPosition;

	H = ViaPosition;
	H(3,1) = 0.0;
	P1 = StartPosition;
	P1(3,1) = 0.0;

	P2(1,1) = 0.0;
	P2(2,1) = -100.0;
	P2(3,1) = 0.0;
	
        V1 = (P1 - H);
        V2 = (P2 - H);
	
		

	Dot = V1(1,1)*V2(1,1) + V1(2,1)*V2(2,1);
	Det = V1(1,1)*V2(2,1) - V1(2,1)*V2(1,1);

 	HomeAngle = (180/PI)* atan2(Det,Dot);  //measured CW from vertical
 	posn = ViaPosition + (TargetAngleVector(FollowAngle+HomeAngle) * TargetDistance);
	attr = TargetColor;
	if ( ( StateGraphics > STATE_MOVING1 ) || ( FieldType == FIELD_PMOVE ) )
	{
		attr = TargetHitColor;
	}

	if(ColourCuedFaster) //For the speed condition, use colour to cue the correct follow-through speed
	{
		attr = FollowCueColor;
        FollowSpeedTarget = FollowSpeedQuickTarget;
        
	}
    else
    {
        FollowSpeedTarget = FollowSpeedSlowTarget;
    }

	// Always display the target
	if( ContextType == TARGET_STATIC_ON )
    {
		GRAPHICS_Circle(&posn,TargetRadius,attr);
	}
    // Display the secondary target as only a visual cue
    if( ContextType == TARGET_STATIC_OFF )
    {
	GRAPHICS_Circle(&posn,TargetRadius,TargetColor);
    }
	// Display the target once the cursor has moved far enough
	if( ContextType == TARGET_APPEAR )
	{
		if ( ( P(2,1) >= VisibleDistance ) || ( PassedVisibleDistance ) )
		{
			PassedVisibleDistance = TRUE;
			GRAPHICS_Circle(&posn,TargetRadius,attr);
		}
	}
	
	// Keep the final target visible all the time (previously it disappeared after the visible distance was passed)
	if(( ContextType == TARGET_STOP ) || ( ContextType == TARGET_STOP_WARNING ))
	{
		attr = YELLOW;
		GRAPHICS_Circle(&posn,TargetRadius,attr);
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

        // Display eye tracker graphics if we're in the eye tracker state. (12)
        if( StateGraphics == STATE_EYETRACKER )
        {
            EYET_GraphicsDisplay();
            continue;
        }

        // Display home position at start of trial.
        if( (StateGraphics >= STATE_SETUP) && (StateGraphics <= STATE_INTERTRIAL) && (FieldType != FIELD_PMOVE) )
        {
            attr = RobotHome() ? HomeColor : NotHomeColor;

            posn = StartPosition;
            posn(3,1) = 0.0;

            GRAPHICS_Circle(&posn,HomeRadius,attr);
        }

	if ( (FieldType == FIELD_PMOVE) && (StateGraphics > STATE_SETUP) )
	{
	    attr = RobotHome() ? HomeColor : NotHomeColor;	    
	    posn = FinishPosition;
	    posn(3,1) = 0.0;
     	    GRAPHICS_Circle(&posn,HomeRadius,attr);
	}

        // Display target spheres when trial running.
        // Should the target go off when the movement finishes?
        if( ( StateGraphics >= STATE_START ) && ( StateGraphics <= STATE_INTERTRIAL ) )
        {
            
			if ( FieldType != FIELD_PMOVE )
			{
				// Display target for movement.
				GraphicsDisplayTarget();
			
				// Display via point for movement.
				posn = ViaPosition;
			        posn(3,1) = 0.0;
				attr = ViaColor;
				if(( ContextType == TARGET_STOP ) || ( ContextType == TARGET_STOP_WARNING ))
				{
				        attr = ImagineViaColor;
				}
				
			    GRAPHICS_Circle(&posn,ViaRadius,attr);
			}


        }

        if( (StateGraphics >= STATE_HOME) && (StateGraphics <= STATE_FEEDBACK) && (FieldType != FIELD_PMOVE) && FixateRequiredFlag )
        {
            FixateCrossPosition(3,1) = 2.0;
            GRAPHICS_FixationCross(FixateCrossPosition,FixateCrossSize,FixateCrossWidth,FixateFlag ? WHITE : BLACK);
        }

        // Display finish position.
        if( (MovementType == MOVETYPE_OUTANDBACK) && ((StateGraphics > STATE_MOVING1) && (StateGraphics <= STATE_INTERTRIAL)) )
        {
            posn = FinishPosition;
            posn(3,1) = 0.0;

            attr = RobotHome() ? HomeColor : NotHomeColor;

            GRAPHICS_Circle(&posn,HomeRadius,attr);
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

void GraphicsDisplayTargetTest( void )
{
int item,attr,target;
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

        // Display home position...
        posn = HomePosition;
        GRAPHICS_Sphere(&posn,HomeRadius,HomeColor);

        // Diplay via point position
        posn = ViaPosition;
        GRAPHICS_Sphere(&posn,ViaRadius,ViaColor);	
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

        if( TargetTestFlag )
        {
            GraphicsDisplayTargetTest();
        }
        else
        {
            GraphicsDisplay();
        }
    }

    Sleep(0);
}

/******************************************************************************/

void GraphicsKeyboard( unsigned char key, int x, int y )
{
    // Process keyboard input.
    switch( toupper(key) )
    {
        case ESC : 
            ProgramExit();
            break;

        case 'c' : 
        case 'C' : 
            // Eye tracker calibration if 'C' key is pressed before a trial has started. (13)
            if( ((State == STATE_SETUP) || (State == STATE_HOME)) && EyeTrackerFlag )
            {
                EYET_CalibrateStart(TRUE);
                StateNext(STATE_EYETRACKER);
            }
            break;

/*      case 'A' : 
            ButtonPress = 1;
            break;

        case 'S' : 
            ButtonPress = 2;
            break;

        case 'D' : 
            ButtonPress = 3;
            break; */

        case 'F' :
            ButtonPress = 4;   // only using a single button to indicate end of imagery.
            break;

        default :
           // Eye tracker keyboard call-back function. (14)
           EYET_KeyEvent(key);
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
    GRAPHICS_OpenGL(GRAPHICS_FLAG_NONE,LIGHTBLUE);

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
    // Load the first (and possibly the only) configuration file.
    if( !ConfigLoad(ConfigFileList[0]) )
    {
        return(FALSE);
    }
    
    // Open list of wave files.
    if( !WAVELIST_Open(WaveList) )
    {
        printf("WAVELIST: Cannot load WAV files.\n");
        return(FALSE);
    }

	ContextFullMovementTimeData.Data(PostMoveDelayInit);

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
    TrialData.AddVariable(VAR(ContextType));
    TrialData.AddVariable(VAR(ContextConstants),FIELD_CONSTANTS);
    TrialData.AddVariable("Trial",TrialNumber); // For saving miss trials.
    TrialData.AddVariable(VAR(MissTrialFlag));  // For saving miss trials.
    TrialData.AddVariable(VAR(TrialDelay));
    TrialData.AddVariable(VAR(InterTrialDelay));
    TrialData.AddVariable(VAR(TargetIndex));
    TrialData.AddVariable(VAR(TargetAngle));
    TrialData.AddVariable(VAR(TargetPosition));
    TrialData.AddVariable(VAR(TargetDistance));
    TrialData.AddVariable(VAR(ViaPosition));
    TrialData.AddVariable(VAR(FixateCrossPosition));
    TrialData.AddVariable(VAR(HomeAngle));
    //TrialData.AddVariable(VAR(WallPosition)); 
    TrialData.AddVariable(VAR(WallDistance)); 
    TrialData.AddVariable(VAR(HomePosition));
    TrialData.AddVariable(VAR(StartPosition));
    TrialData.AddVariable(VAR(FinishPosition));
    TrialData.AddVariable(VAR(MissTrials));
    TrialData.AddVariable(VAR(MissTrialsFixation));
    TrialData.AddVariable(VAR(TrialDuration));
    TrialData.AddVariable(VAR(MovementReactionTime));
    TrialData.AddVariable(VAR(MovementDurationTime));
    TrialData.AddVariable(VAR(MovementDurationToViaTime));
    TrialData.AddVariable(VAR(MovementDurationTooFast));
    TrialData.AddVariable(VAR(MovementDurationTooSlow));
    TrialData.AddVariable(VAR(MovementDurationTooSlowToVia));
    TrialData.AddVariable(VAR(PMoveStartPosition));
    TrialData.AddVariable(VAR(PMoveEndPosition));
    TrialData.AddVariable(VAR(ViaToleranceTime));
    TrialData.AddVariable(VAR(ViaTimeOutTime));
    TrialData.AddVariable(VAR(ViaSpeedThreshold));
    TrialData.AddVariable(VAR(FollowSpeedQuickTarget));
    TrialData.AddVariable(VAR(FollowSpeedSlowTarget));
    TrialData.AddVariable(VAR(FollowSpeedTolerance));
    TrialData.AddVariable(VAR(FollowSpeedTarget));
    TrialData.AddVariable(VAR(PostMoveDelayInit));
    TrialData.AddVariable(VAR(PostMoveDelayTime));
	
    // Add each variable to the FrameData matrix.
    FrameData.AddVariable(VAR(TrialTime));         
    FrameData.AddVariable(VAR(State));
    FrameData.AddVariable(VAR(StateGraphics));
    FrameData.AddVariable(VAR(ForcesFunctionLatency));
    FrameData.AddVariable(VAR(ForcesFunctionPeriod));
    FrameData.AddVariable(VAR(RobotPosition));
    FrameData.AddVariable(VAR(RobotVelocity));     
    FrameData.AddVariable(VAR(RobotForces));       
    FrameData.AddVariable(VAR(RobotActiveFlag));
    FrameData.AddVariable(VAR(HandleForces));
    FrameData.AddVariable(VAR(HandleTorques));
    FrameData.AddVariable(VAR(CursorPosition));
    FrameData.AddVariable(VAR(PMoveState));
    FrameData.AddVariable(VAR(PMoveStateTime));
    FrameData.AddVariable(VAR(PMoveStateRampValue));
    FrameData.AddVariable(VAR(PMoveStatePosition));

    // Eye tracker frame data variables if required. (15)
    if( EyeTrackerFlag )
    {
        FrameData.AddVariable(VAR(EyeTrackerFrameCount));
        FrameData.AddVariable(VAR(EyeTrackerTimeStamp));
        FrameData.AddVariable(VAR(EyeTrackerEyeXY),2);
        FrameData.AddVariable(VAR(EyeTrackerPupilSize));
    }

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
double A,O;
static matrix H(3,1), V1(3,1), V2(3,1), P1(3,1), P2(3,1);
static double Dot, Det;

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
        
        HomePosition = FieldHomePosition[FieldIndex]; 

        TargetAngle = ContextConstants[0];  
	H = ViaPosition;
	H(3,1) = 0.0;
	P1 = HomePosition;
	P1(3,1) = 0.0;

	P2(1,1) = 0.0;
	P2(2,1) = -100.0;
	P2(3,1) = 0.0;
	
        V1 = (P1 - H);
        V2 = (P2 - H);
	
	Dot = V1(1,1)*V2(1,1) + V1(2,1)*V2(2,1);
	Det = V1(1,1)*V2(2,1) - V1(2,1)*V2(1,1);

 	HomeAngle = (180/PI)* atan2(Det,Dot);     
		
		// Target angle and target position.
		if (( ContextType == TARGET_STATIC_ON ) || (ContextType == TARGET_APPEAR ))
		{
			TargetPosition = ViaPosition + (TargetAngleVector(TargetAngle+HomeAngle) * TargetDistance); 
		}
		else
		{
            TargetPosition = ViaPosition;
		}


        // Start and finish position of movement.
        StartPosition = HomePosition;

        switch( MovementType )
        {
            case MOVETYPE_OUTANDBACK :
                FinishPosition = HomePosition;
                break;

            case MOVETYPE_OUTTHENBACK :
            case MOVETYPE_OUTONLY :
                FinishPosition = TargetPosition;
                break;
        }

        // Move direction (Out,Back,OutAndBack).
        MovementDirection = MoveTypeDirection[MovementType];

        FixateCrossPosition = ViaPosition;

        // Save TrialData row for this trial.
        TrialData.RowSave(TrialOffset+Trial);

        // Increment trial number depending on movement type...
        Trial += MoveTypeTrials[MovementType];
    }

    if( !ok )
    { 
        return(ok);
    }

/*  Creating "back" movements has been moved to TrialList() function (JNI 31/May/2016).
    // Generate "back" movements if not "OutAndBack" paradigm.
    if( MovementType != MOVETYPE_OUTANDBACK )
    {
        for( Trial=1; (Trial <= Trials); ) 
        {
            // Load the next odd-number trial to retrieve the home position.
            i = TrialOffset+Trial+2;
            
	       
	    if( i < (TrialOffset+Trials+1) ) // Only load trial if it exists within this phase
            { 
   	        // printf("DEBUG i= %d, Trial= %d, Trialoffset= %d, Trials= %d\n",i,Trial,TrialOffset,Trials);

	        TrialData.RowLoad(i);
                FinishPositionTemp = HomePosition;
            }
            else
            {
		if (ConfigIndex == (ConfigFileCount-1) )  // if this is the last trial and last phase, return instead to the viapoint.
                {
		  // printf("LAST TRIAL= %d\n", i);
                   FinishPositionTemp.zeros();
		}
		else 
		{
   	          // printf("END OF PHASE BUT NOT LAST, i= %d, Trial= %d, Trialoffset= %d, Trials= %d\n",i,Trial,TrialOffset,Trials);

	           TrialData.RowLoad(i); 		// last trial but not last phase, continue return to next home position.
                   FinishPositionTemp = HomePosition;
		}            
	    }

            // Load the odd-number trial.
            TrialData.RowLoad(TrialOffset+Trial);

            // Increment it to be an even-number trial.
            Trial++;

            FieldType = FIELD_PMOVE;
            FieldTrials[FieldType]++;

            FieldAngle = 0.0;

            for( i=0; (i < FIELD_CONSTANTS); i++ )
            {
                FieldConstants[i] = 0.0;
            }	

            MovementDirection = MOVEDIR_BACK;

            // Set finish position to home position of the next trial.
            FinishPosition = FinishPositionTemp;
            

	    printf("FinishPosition= %.1f,%.1f,%.1f\n",FinishPosition(1,1), FinishPosition(2,1), FinishPosition(3,1));


            TrialData.RowSave(TrialOffset+Trial);
            Trial++;
        }
    }*/

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
                RestBreakTrials[RestBreakCount++] = TotalTrials;
                printf("RestBreakTrials[%d] = %d\n",RestBreakTrials,TotalTrials);
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

    TotalTrials = 0;
    for( i=0; (i < FIELD_MAX); i++ )
    {
        FieldTrials[i] = 0;
    }

    printf("Making list of %d trials (ESCape to abort)...\n",TotalTrials);

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

            // Set finish position of this trial to home position of the next trial.
            FinishPosition = HomePosition;
          
            // Save the even numbered "back" trial and decrement trial number.
            TrialData.RowSave(Trial);
            Trial--;
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
    ExperimentTimer.Reset();
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

