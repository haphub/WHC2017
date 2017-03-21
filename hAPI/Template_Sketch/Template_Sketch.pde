
/**
 *******************************************************************************
 * @file       Pantagraph_Processing.pde
 * @author     
 * @version    V0.1.0
 * @date       27-February-2017
 * @brief      Prototype tests for encoder input for Pantagraph application
 *******************************************************************************
 * @attention
 *
 *
 *******************************************************************************
 */

/* library imports *************************************************************/
import processing.serial.*;

/* Device block definitions ************************************/




/* Thread  Parameters ************************************/
int             baseFrameRate     = 1000;
int             animation_count   = baseFrameRate/501;
int             haptics_count     = baseFrameRate/1000;


/*Graphic Objects *******************************************/



/*Physics parameters*/



// generic data for a 2DOF device



/**
 * @brief    Main setup function, defines parameters and hardware setup
 */
void setup(){
  
  /*Setup for the graphic display window and drawing objects*/
  
  size(600, 400, P2D);
  background(255);
  frameRate(baseFrameRate);

  
  /* Initialization of the Board, Device, and Device Components*/ 
  
  //BOARD INITIALIZATION
  
  //DEVICE INITIALIZATION

 
}

      
/**
 * @brief    Main draw function, updates frame at perscribed frame rate
 */
void draw(){
  
 
  if(/*data_available*/){

/*** GET END-EFFECTOR POSITION (TASK SPACE)****/ 
 
    //INSERT CODE HERE
 
/************************************/   
		
/*** PHYSICS OF THE SIMULATION ****/ 

  //INSERT CODE HERE

/************************************/

  }

	/******* ANIMATION TIMER ********/ 
  if(frameCount % animation_count == 0){
      //INSERT GRAPHICS CODE HERE
  }
	
  /********** HAPTICS TIMER *************/ 
	
  if(frameCount % haptics_count == 0){
     //HAPTICS CODE HERE
  }
  
  
}