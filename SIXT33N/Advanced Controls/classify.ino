/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and K-means constants
#define SNIPPET_SIZE                  130
#define PRELENGTH                     5
#define THRESHOLD                     0.5

#define KMEANS_THRESHOLD            .06
#define LOUDNESS_THRESHOLD          600

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[130] = {0.0209264380898, 0.0285890756235, 0.0337535017905, 0.0253297960253, 0.00199136114908, -0.0106479138388, -0.0270110168093, -0.0239379143903, -0.0233171640712, 0.0082187137926, 0.0355248519683, 0.0655229050032, 0.0815172270351, 0.087477238695, 0.10892612627, 0.128776437902, 0.13657455943, 0.148692077338, 0.16140750855, 0.154772756061, 0.173204529738, 0.188581419753, 0.20647553719, 0.230509232017, 0.242770033891, 0.239535752775, 0.242903481257, 0.233486733585, 0.219802089322, 0.201424414072, 0.192955395223, 0.161848950954, 0.140056857034, 0.12043565376, 0.0844208007167, 0.0627973599224, 0.042340013763, 0.0280228020806, 0.00486978263431, -0.0330642387563, -0.0526082886484, -0.0538371958873, -0.0829078883502, -0.0795577717668, -0.083565021683, -0.0766929583815, -0.0864735942664, -0.0862899194357, -0.0793863859405, -0.0935225378869, -0.102987548026, -0.0990033243867, -0.0921358124375, -0.0842073343211, -0.0814338962727, -0.0790842416312, -0.0714608323061, -0.0621297964713, -0.0483695677557, -0.0380622033946, -0.037348087004, -0.0293168510849, -0.0295992913259, -0.0241291027898, -0.0171316554281, -0.0163850994692, -0.00960604671367, -0.00743921164339, -0.00919348038197, -0.0144301591583, -0.0227963916905, -0.0360395460146, -0.0500449237705, -0.0598942435536, -0.0692678341334, -0.0735190273099, -0.0773032807732, -0.0808488325637, -0.0818180424996, -0.0850222893283, -0.0870434831535, -0.0842758896973, -0.0842059983129, -0.0838119002558, -0.0795512303862, -0.0762442889064, -0.0713920401034, -0.0720750677951, -0.0712124859597, -0.0655555416588, -0.0604333700419, -0.0557280753149, -0.0526104013658, -0.0475704918742, -0.0422081732057, -0.0398051321125, -0.0372555023466, -0.0337300094529, -0.0285650478875, -0.0264237049177, -0.0229226703425, -0.0206328874976, -0.0172525984287, -0.014758588272, -0.0149390444419, -0.016381039909, -0.0160730345627, -0.0154042155946, -0.0171397364715, -0.022339575911, -0.0220658141968, -0.0217224910022, -0.0225317152642, -0.0258011001549, -0.0257880255391, -0.025407501499, -0.0273294531064, -0.0236194428735, -0.021661284407, -0.021010463598, -0.0186326793076, -0.0171861393395, -0.0149738702706, -0.0150803742756, -0.0151767155646, -0.0148854254559, -0.0146466505069, -0.0133087122161, -0.0109841393391, -0.0122604265616};
float pca_vec2[130] = {0.0207213236963, 0.0245939469947, 0.0309214863319, 0.0393150180876, 0.0746176315979, 0.129759915013, 0.146156235965, 0.164467520453, 0.180878897308, 0.205102850589, 0.214193088359, 0.226142742584, 0.220697884303, 0.207651183783, 0.203276692331, 0.193408228341, 0.161232600842, 0.164443207568, 0.149687739806, 0.102963565982, 0.0840617705299, 0.0438824460475, -0.0221964529906, -0.0555171615293, -0.0571773662315, -0.0720578889079, -0.0685171222883, -0.0766972047649, -0.0994069461465, -0.0961808738185, -0.142553652635, -0.156000339393, -0.159603281623, -0.148665066739, -0.151647339183, -0.173637671822, -0.176579197965, -0.15755980537, -0.159972071517, -0.157201657422, -0.148339491577, -0.108508279944, -0.115097267058, -0.110790463691, -0.0998342854471, -0.0840536355726, -0.0768786444268, -0.0598671410187, -0.0549890011393, -0.0514027387024, -0.0648351850675, -0.0613589014687, -0.0542436141187, -0.0411463931256, -0.044439430962, -0.0476971092321, -0.0483378326061, -0.0514340054761, -0.0584494220406, -0.0618243467261, -0.0492511107175, -0.0525470385475, -0.0723420107661, -0.0619868976583, -0.0482857773058, -0.0484495075308, -0.0308729144133, -0.0308214152211, -0.0241780211015, -0.0233324820078, -0.0103343792913, 0.0100977486528, 0.0260188475114, 0.0320187649124, 0.0402855423321, 0.0468550068739, 0.0464033957894, 0.052405071279, 0.0535819669552, 0.0561696605281, 0.0625850158991, 0.0587285461918, 0.0559029063304, 0.0534887277295, 0.0514040719918, 0.0491524648325, 0.0484930915161, 0.0485544831534, 0.044967773979, 0.0423581158248, 0.0325763133211, 0.0301148177875, 0.02417981057, 0.029046400086, 0.0257826125319, 0.0254065293778, 0.026834574402, 0.0224447135325, 0.0185032187474, 0.0177631661362, 0.0130282657976, 0.0151235644734, 0.0110472094392, 0.00772582169774, 0.00624687265426, 0.00581044831555, 0.00304110570535, 0.00283202749309, -0.000600980985392, -0.00243509621327, -0.00672463452167, -0.00362739624182, -0.0070723081056, -0.00780177017013, -0.0111428163102, -0.0120832588438, -0.0133367955245, -0.0132289273464, -0.0103654995586, -0.011051903978, -0.00918210318043, -0.00984297251028, -0.00744437795235, -0.00582403920611, -0.00609969718711, -0.00473937481612, -0.00260648528294, -0.00215854751063, -0.00582906991006, -0.00485675120034};
float projected_mean_vec[2] = {0.0317883799486, 0.020217390534};
float centroid1[2] = {0.0620438527193, -0.00657159855303};
float centroid2[2] = {-0.0256892619825, -0.0055353214533};
float centroid3[2] = {-0.0305247443876, 0.0250517219304};
float centroid4[2] = {-0.00582984634914, -0.0129448019241};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i] * pca_vec1[i];
          proj2 += result[i] * pca_vec2[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[0];
      proj2 -= projected_mean_vec[1];

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      for (int i = 0; i < 4; i++) {
        float dist = l2_norm(proj1, proj2, centroids[i]);
          if(dist < best_dist){
              best_index = i;
              best_dist = dist;
          }
      }


      // Compare 'best_dist' against the 'KMEANS_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'KMEANS_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      if(best_dist < KMEANS_THRESHOLD){
          if(best_index == 0){
              Serial.println("Hi");
          }
          if(best_index == 1){
              Serial.println("Gogogogo");
          }
          if(best_index == 2){
              Serial.println("Butterfly");
          }
          if(best_index == 3){
              Serial.println("Jaguar");
          }
      } else {
        Serial.println("noise");
      }


      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }


    delay(2000);
    re_pointer = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
