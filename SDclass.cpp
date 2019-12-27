#include "SDclass.h"

char REDpath[13] = "PATH_RED.txt";
char REDvel[13] = "VEL_RED.txt";
char BLUpath[13] = "PATH_BLU.txt";
char BLUvel[13] = "VEL_BLU.txt";

mySDclass::mySDclass(){
}

/* SDカードの初期化 */
int mySDclass::init(){
  Serial.println("mySDclass");

  if (SD.begin()) {
    SD_enable = true;
    Serial.println("return 0");
    Serial.flush();
    return 0;
  }

  Serial.println("return -1");
  Serial.flush();
  return -1;
}

/* ログデータ書き出し用ファイル名の設定 */
int mySDclass::make_logfile(){
  bool nameOK = false;
  int file_num = 0;

  while(!nameOK && file_num < 10000){
    logFileName = "MR1_";
    if(file_num < 10){
      logFileName += "000";
      logFileName += String(file_num);
    }else if(file_num < 100){
      logFileName += "00";
      logFileName += String(file_num);
    }else if(file_num < 1000){
      logFileName += "0";
      logFileName += String(file_num);
    }else{
      logFileName += String(file_num);
    }
    
    logFileName += ".txt";
    logFileName.toCharArray(c_logFileName, 13 );
    if(SD.exists(c_logFileName)){
      file_num++;
    }
    else{
      nameOK = true;
    }
  }
  return 0;
}

/* ログデータ書き出し用の関数 */
int mySDclass::write_logdata(String dataString){
  File dataFile = SD.open(c_logFileName, FILE_WRITE);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    //Serial.println(dataString);
    return 0;
  }  
  else {
    //Serial.println("error opening ");
    return -1;
  }
}

int mySDclass::path_read(int field, double Px[], double Py[], double vel[], double angle[], int mode[], int count[], double tbe[]){
  File myFile;
  char *pathFile;
  char *velFile;
  char tmpchar;
  char tmpA[10], tmpB[10], tmpC[10], tmpD[10], tmpE[10];
  bool file_end  = false;
  int numa = 0, numb = 0, numc = 0, numd = 0, nume = 0;
  int path_num = 0, point_num = 0;

  // 赤か青かで読み込むファイルを変更する
  if(field == RED){
    pathFile = REDpath; // 赤ゾーンのパス設定ファイル
    velFile = REDvel; // 赤ゾーンの速度と角度の設定ファイル
  }else if(field == BLUE){
    pathFile = BLUpath; // 青ゾーンのパス設定ファイル
    velFile = BLUvel; // 青ゾーンの速度と角度の設定ファイル
  }else{
    return -1;
  }

  // パス設定用ファイルからデータを読み込む
  myFile = SD.open(pathFile, FILE_READ);
  if (myFile) {
    // read from the file until there's nothing else in it:
    while (!file_end && myFile.available()) {
      while((tmpchar = myFile.read()) != ','){ // カンマが来るまで繰り返し
        tmpA[numa] = tmpchar; // 文字列に1文字ずつ格納していく
        numa++;
      }
      *Px = str2double(tmpA, numa); //関数でdoubleに変換
      //Serial.print(tmpA);
      Serial.print(*Px);
      for(int i = 0; i < 10; i++) tmpA[i] = 0; // 文字列を初期化
      numa=0;
      Px++;
      while(((tmpchar = myFile.read()) != '\r' && tmpchar != ';') && tmpchar != '/'){ // 改行コードかセミコロン，スラッシュが来るまで繰り返し
        tmpB[numb] = tmpchar;
        numb++;
      }
      if(tmpchar == ';'){
        file_end = true;
      }else if(tmpchar == '/'){ // コメントアウト対応
        while(tmpchar = myFile.read() != '\n');
      }else{
        myFile.read(); // "\n"を捨てるため
      }
      *Py = str2double(tmpB, numb); //関数でdoubleに変換
      point_num++;

      Serial.print(",");
      Serial.println(*Py);
      //Serial.println(tmpB);
      for(int i = 0; i < 10; i++) tmpB[i] = 0;
      numb = 0;
      Py++;
    }
    //Serial.print("path done! ");
    file_end = false;
    //   the file:
     myFile.close();

     Serial.flush();
  } else {
  	// if the file didn't open, print an error:
    //Serial.println("error opening test.txt");
    return -2;
  }

  // 速度設定用ファイルからデータを読み込む
  myFile = SD.open(velFile, FILE_READ);
  if (myFile) {
    while (!file_end && myFile.available()) {
      while((tmpchar = myFile.read()) != ','){
        tmpA[numa] = tmpchar;
        numa++;
      }
      *vel = str2double(tmpA, numa); //関数でdoubleに変換
      //Serial.print(tmpA);
      Serial.print(*vel);
      for(int i = 0; i < 10; i++) tmpA[i] = 0;
      numa = 0;
      vel++;
      //////////////////////////////////
      while((tmpchar = myFile.read()) != ','){
        tmpB[numb] = tmpchar;
        numb++;
      }
      *angle = str2double(tmpB, numb); //関数でdoubleに変換
      //Serial.print(tmpA);
      Serial.print(",");
      Serial.print(*angle);
      for(int i = 0; i < 10; i++) tmpB[i] = 0;
      numb = 0;
      angle++;
      //////////////////////////////////
      while((tmpchar = myFile.read()) != ','){
        tmpC[numc] = tmpchar;
        numc++;
      }
      *mode = str2uint(tmpC, numc); //関数でdoubleに変換
      //Serial.print(tmpA);
      Serial.print(",");
      Serial.print(*mode);
      for(int i = 0; i < 10; i++) tmpC[i] = 0;
      numc = 0;
      mode++;
      //////////////////////////////////
      while((tmpchar = myFile.read()) != ','){
        tmpD[numd] = tmpchar;
        numd++;
      }
      *count = str2uint(tmpD, numd); //関数でdoubleに変換
      //Serial.print(tmpA);
      Serial.print(",");
      Serial.print(*count);
      for(int i = 0; i < 10; i++) tmpD[i] = 0;
      numd = 0;
      count++;
      //////////////////////////////////
      while((tmpchar = myFile.read()) != '\r' && tmpchar != ';'){
        tmpE[nume] = tmpchar;
        nume++;
      }
      if(tmpchar == ';'){
        file_end = true;
      }else{
        myFile.read(); // "\n"を捨てるため
      }
      *tbe = str2double(tmpE, nume); //関数でdoubleに変換
      path_num++;

      //Serial.print(tmpB);
      Serial.print(",");
      Serial.println(*tbe);
      for(int i = 0; i < 10; i++) tmpE[i] = 0;
      nume = 0;
      tbe++;
    }
    //Serial.println("vel/angle done!");
    // close the file:
     myFile.close();

     Serial.flush();
  } else {
  	// if the file didn't open, print an error:
    //Serial.println("error opening test.txt");
    return -3;
  }

  if((int)((point_num - 2) / 3) >= (path_num - 1)){
    return path_num - 1;
  }
  return -4;
}

double mySDclass::str2double(char* str, int num){
  double ret = 0.0;
  bool minus = false;
  int m = 0, keta;
  
  // マイナス符号が付いているかチェック
  if(str[0] == '-'){
    minus = true;
    m++;
  }

  // 何桁あるかを確認
  keta = m;
  while((str[keta] != '.') && (keta < num)){
    keta++;
  }
  keta = keta - (m + 1);
  
  // 整数部を変換
  for(int i = m; i <= (keta + m); i++){
    if(str[i] >= 48 && str[i] <= 57){
      ret += (double)(str[i] - 48) * pow(10.0, keta - (i - m));
    }
  }
  
  // 小数部を変換
  int n = -1;
  for(int i = keta + m + 2; i < num; i++ ){
    if(str[i] >= 48 && str[i] <= 57){
      ret += (double)(str[i] - 48) * pow(10.0, n);
      n--;
    }
  }
  if(minus) return -1.0 * ret;
  return ret;
}

int mySDclass::str2uint(char* str, int num){
  num--;
  int ret = 0;
  //bool minus = false;
  
  // 整数部を変換
  for(int i = 0; i <= num; i++){
    if(str[i] >= 48 && str[i] <= 57){
      ret += (double)(str[i] - 48) * pow(10.0, num - i);
    }
  }
  
  return ret;
}
