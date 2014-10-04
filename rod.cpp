#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

// 描画関数の選択

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawLine     dsDrawLineD
#endif

// シミュレーション環境変数

dWorldID world;  // 動力学計算用ワールド
dSpaceID space;  // 衝突検出用スペース
dGeomID  ground; // 地面
dJointGroupID contactgroup; // コンタクトグループ
dsFunctions fn;
dMass mass; // 質量パラメータ
typedef struct {       // MyObject構造体
  dBodyID body;        // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;        // ジオメトリのID番号(衝突検出計算用）
  double  l,r,m;       // 長さ[m], 半径[m]，質量[kg]
} MyObject;
MyObject rod[3];          // ロッド（0:SHIELD, 1:ROD, 2:BODY）
dJointID rod_joint[3];    // ロッドジョイント
MyObject bullet;          // 銃弾オブジェクト
MyObject target;          // ターゲット
dMatrix3 R;               // 回転行列


// オブジェクト定数

//#define CANNON_LENGTH 1.0  // 砲台（立方体）一辺（1.0m）
//#define CANNON_WEIGHT 10.0 // 砲台重さ（kg）
#define CANNON_X 10.0       // 砲台中心座標(X)
#define CANNON_Y 20.0//50.0       // 砲台中心座標(Y)
#define CANNON_Z 0.6       // 砲台中心座標(Z)
//#define TURRET_LENGTH 1.0  // 砲塔長さ
//#define TURRET_WIDTH 0.25  // 砲塔太さ（正方形）
//#define TURRET_WEIGHT 1.0  // 砲塔重さ
//#define TURRET_X 0.0       // 砲塔中心座標(X)
//#define TURRET_Y 4.5       // 砲塔中心座標(Y)
//#define TURRET_Z 0.5       // 砲塔中心座標(Z)
#define BULLET_RADIUS 0.025// 弾丸半径（球形：25mm）
#define BULLET_WEIGHT 0.01 // 弾丸重さ（10g）
#define ROD_LENGTH 1.6     // ロッドの長さ（1本で両端までの長さ）
#define ROD_WIDTH 0.10     // ロッドの太さ（正方形）
#define ROD_WEIGHT 1.0     // ロッド重さ（1kg）
#define SHIELD_RADIUS 0.10 // ロッドベースの半径
#define SHIELD_LENGTH 0.15 // ロッドベースの厚み
#define SHIELD_WEIGHT 1.0  // ロッドベース重さ
#define SHIELD_X 0.0       // ロッドベース中心座標(X)
#define SHIELD_Y 0.0       // ロッドベース中心座標(Y)
#define SHIELD_Z 1.1       // ロッドベース中心座標(Z)
#define BODY_LENGTH 0.2    // 本体奥行
#define BODY_WIDTH 0.7     // 本体幅
#define BODY_HEIGHT 1.8    // 本体高さ
#define BODY_WEIGHT 70.0   // 本体重さ
#define BODY_X 0.0         // 本体中心座標(X)
#define BODY_Y -0.5        // 本体中心座標(Y)
#define BODY_Z 0.9         // 本体中心座標(Z)

// 制御用変数

dReal rod_q_d = 0.0;              // ロッドの目標角度
dReal cannon_q_d[2] = {0.0, 0.0}; // 砲台の目標角度（0:水平, 1:仰角）
dReal diff_sum = 0.0;             // 偏差のsum
dReal mytime = 0.0;               // 制御時間
bool act_flg = false;              // 追従スイッチ


// ロボットの生成
void create() {

  // SHIELDの生成（空間に固定）
  rod[0].body = dBodyCreate(world);
  dBodySetPosition(rod[0].body, SHIELD_X, SHIELD_Y, SHIELD_Z);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass, SHIELD_WEIGHT, 2, SHIELD_RADIUS, SHIELD_LENGTH);
  dBodySetMass(rod[0].body, &mass);
  rod[0].geom = dCreateCylinder(space, SHIELD_RADIUS, SHIELD_LENGTH);
  dGeomSetBody(rod[0].geom, rod[0].body);
  dRFromAxisAndAngle(R, 1, 0, 0, 0.5 * M_PI);  // x軸に90度回転
  dBodySetRotation(rod[0].body, R);

  rod_joint[0] = dJointCreateFixed(world, 0);  // 固定ジョイント
  dJointAttach(rod_joint[0], rod[0].body, 0);
  dJointSetFixed(rod_joint[0]);

  // RODの生成（回転ジョイントy軸に回転軸）
  rod[1].body = dBodyCreate(world);
  dBodySetPosition(rod[1].body, SHIELD_X, SHIELD_Y, SHIELD_Z);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, ROD_WEIGHT, ROD_WIDTH, ROD_WIDTH, ROD_LENGTH);
  dBodySetMass(rod[1].body, &mass);
  rod[1].geom = dCreateBox(space, ROD_WIDTH, ROD_WIDTH, ROD_LENGTH);
  dGeomSetBody(rod[1].geom, rod[1].body);
  dRFromAxisAndAngle(R, 0, 0, 1, 0.25 * M_PI);  // z軸に45度回転
  dBodySetRotation(rod[1].body, R);

  rod_joint[1] = dJointCreateHinge(world, 0); // ヒンジジョイント
  dJointAttach(rod_joint[1], rod[1].body, rod[0].body);
  dJointSetHingeAnchor(rod_joint[1], SHIELD_X, SHIELD_Y, SHIELD_Z);
  dJointSetHingeAxis(rod_joint[1], 0, 1, 0);// y軸ジョイント

  // BODYの生成（たてておくだけ）
  rod[2].body = dBodyCreate(world);
  dBodySetPosition(rod[2].body, BODY_X, BODY_Y, BODY_Z);
  dMassSetZero(&mass);
  dMassSetBoxTotal(&mass, BODY_WEIGHT, BODY_WIDTH, BODY_LENGTH, BODY_HEIGHT);
  dBodySetMass(rod[2].body, &mass);
  rod[2].geom = dCreateBox(space, BODY_WIDTH, BODY_LENGTH, BODY_HEIGHT);
  dGeomSetBody(rod[2].geom, rod[2].body);

  // BULLETの生成（CANNON中心に初期座標）
  bullet.body = dBodyCreate(world);
  dMassSetZero(&mass);
  dMassSetSphereTotal(&mass, BULLET_WEIGHT, BULLET_RADIUS);
  dBodySetMass(bullet.body,&mass);
  dBodySetPosition(bullet.body, CANNON_X, CANNON_Y, CANNON_Z);
  bullet.geom = dCreateSphere(space, BULLET_RADIUS);
  dGeomSetBody(bullet.geom, bullet.body);

  // TARGETの生成
//  target.body = dBodyCreate(world);
//  dMassSetZero(&mass);
//  dMassSetSphereTotal(&mass, 0.0001, BULLET_RADIUS);
//  dBodySetMass(target.body,&mass);
//  dBodySetPosition(target.body, SHIELD_X, SHIELD_Y, SHIELD_Z);
//  target.geom = dCreateSphere(space, BULLET_RADIUS);
//  dGeomSetBody(target.geom, target.body);

}

// ロボットの描画
static void draw() {
  const dReal *pos, *rot;
  dReal side[3];

  // SHIELDの描画
  pos = dBodyGetPosition(rod[0].body);
  rot = dBodyGetRotation(rod[0].body);
  dsSetColor(1.0 ,0.0 ,0.0);
  dsDrawCylinder(pos, rot, SHIELD_LENGTH, SHIELD_RADIUS);

  // RODの描画
  pos = dBodyGetPosition(rod[1].body);
  rot = dBodyGetRotation(rod[1].body);
  side[0] = ROD_WIDTH;
  side[1] = ROD_WIDTH;
  side[2] = ROD_LENGTH;
  dsSetColor(0.9 ,0.7 ,0.13);
  dsDrawBox(pos, rot, side);

  // BODYの描画
  pos = dBodyGetPosition(rod[2].body);
  rot = dBodyGetRotation(rod[2].body);
  side[0] = BODY_WIDTH;
  side[1] = BODY_LENGTH;
  side[2] = BODY_HEIGHT;
  dsSetColor(0.0 ,0.0 ,1.0);
  dsDrawBox(pos, rot, side);

  // BULLETの描画
  pos = dBodyGetPosition(bullet.body);
  rot = dBodyGetRotation(bullet.body);
  dsSetColor(0 ,0 ,0);
  dsDrawSphere(pos, rot, BULLET_RADIUS+0.1);

  // RAYの描画
  dVector3 pos_s,pos_e;
  pos_s[0] = CANNON_X; pos_s[1]=CANNON_Y; pos_s[2]=CANNON_Z;
  pos_e[0] = CANNON_X + CANNON_Y * tan(cannon_q_d[0]);
  pos_e[1] = 0.0;
  pos_e[2] = CANNON_Z + CANNON_Y * tan(- 1.0 * cannon_q_d[1]) / cos(cannon_q_d[0]);
  dsSetColor(1 ,0 ,0);
  dsDrawLine(pos_s, pos_e);

}

// コールバック関数
static void nearCallback(void *data, dGeomID o1, dGeomID o2) {
  int i,n;

  dBodyID b1 = dGeomGetBody(o1);
  dBodyID b2 = dGeomGetBody(o2);
  if(b1 && b2 && dAreConnected(b1, b2))
    return;
  const int N = 4;
  dContact contact[N];
  n = dCollide (o1,o2,N,&contact[0].geom,sizeof(dContact));
  if(n > 0) {
    for(i=0; i<n; i++) {
      contact[i].surface.mode = dContactSlip1 | dContactSlip2 | dContactSoftERP | dContactSoftCFM | dContactApprox1;
      if(dGeomGetClass(o1) == dSphereClass || dGeomGetClass(o2) == dSphereClass)
        contact[i].surface.mu = 20;
      else
        contact[i].surface.mu = 0.5;
      contact[i].surface.slip1 = 0.0;
      contact[i].surface.slip2 = 0.0;
      contact[i].surface.soft_erp = 0.8;
      contact[i].surface.soft_cfm = 0.01;
      dJointID c = dJointCreateContact (world,contactgroup,contact+i);
      dJointAttach (c,dGeomGetBody(o1),dGeomGetBody(o2));
    }
  }

}

// 制御式
static void control() {
  mytime += 0.0001;
/*  // ロッドのマニュアル制御

  dReal rod_q = dJointGetHingeAngle(rod_joint[1]);
  dReal rod_input = 5.0 * (rod_q_d - rod_q);
  dJointSetHingeParam(rod_joint[1], dParamVel, rod_input);
  dJointSetHingeParam(rod_joint[1], dParamFMax, 10000.0);
*/
  // 弾丸の位置を取得
  const dReal *pos, *vel;           // 弾丸の位置と速度ベクトル
  dReal tar[3] = {0.0};             // 計算後の着弾予想点
  dReal rot[12] = {1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0};
  //const dReal *tar_pos, *tar_rot;   // 着弾予想地点
  dReal q_p_d = 0.0;                // 広域レーダーからの目標角度
  dReal q_c_d = 0.0;                // 近接センサからの偏差
  dReal distance = 0.0;             // SHIELD中心から弾丸までの距離
  dReal rod_q = dJointGetHingeAngle(rod_joint[1]);  // RODの現在角度
  dReal rod_input = 0.0;                            // ROD制御入力
  pos = dBodyGetPosition(bullet.body);
  vel = dBodyGetLinearVel(bullet.body);
  //pos = dBodyGetRelPointPos(bullet.body);
  //vel = dBodyGetRelPointVel(bullet.body);
  distance = sqrt(
    (SHIELD_X-pos[0])*(SHIELD_X-pos[0]) +
    (SHIELD_Y-pos[1])*(SHIELD_Y-pos[1]) +
    (SHIELD_Z-pos[2])*(SHIELD_Z-pos[2])
  );

  if(abs(vel[1])>0.1){
    tar[0] = pos[0] + pos[1]*vel[0]/abs(vel[1]);
    tar[1] = 0.0;
    tar[2] = pos[2] + pos[1]*vel[2]/abs(vel[1]);
    // 着弾予想点の描画
    dsSetColor(1 ,0 ,0);
    dsDrawSphere(tar, rot, BULLET_RADIUS+0.1);

    // 広域レーダー
    if(distance < 50.0){
      q_p_d = 1.0 * atan(tar[0]/(tar[2]-SHIELD_Z));
    }

    // 近接センサ
    if(distance < 10.0){
      q_c_d = -1.0 * atan(pos[0]/pos[2]);
      diff_sum += q_c_d - rod_q;
    }
  }

  // 入力トルク（疑似付加目標角度）を計算
  if(act_flg){  // スイッチがオンの場合
    rod_input = 500.0 * (q_p_d - rod_q);   // 通常制御のみ
    //rod_input = 50.0 * (q_p_d - rod_q) + 0.001 * diff_sum;
    rod_q_d = rod_q;
    //FILE *fp_0; fp_0 = fopen("dataA.csv","a");
    //fprintf(fp_0,"%lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf,%3.9lf\n", mytime, pos[0], pos[1], pos[2], vel[0], vel[1], vel[2], tar[0], tar[1], tar[2], q_p_d, q_c_d, rod_q);
    //fclose(fp_0);
  }else{
    rod_input = 500.0 * (rod_q_d - rod_q);   // マニュアル操作
  }

  //rod_input = 500.0 * (rod_q_d - rod_q);   // マニュアル操作
  printf("%lf : %lf -> %lf = %lf\r", mytime, rod_q_d, rod_q, rod_input);
  //printf("%lf, %lf, %lf\r", pos[0], pos[1], pos[2] );
  //printf("%lf, %lf, %lf\r", vel[0], vel[1], vel[2] );
  //printf("%lf, %lf, %lf\r", rod_q_d*180.0/M_PI, rod_q*180.0/M_PI, rod_input);


  // 制御入力
  dJointSetHingeParam(rod_joint[1], dParamVel, rod_input);
  dJointSetHingeParam(rod_joint[1], dParamFMax, 100000.0);

}

/*** キー入力関数 ***/
void command(int cmd)
{
  switch(cmd){
  case '0':
    if(act_flg)
      act_flg = false;
    else
      act_flg = true;
    break;
  case 'a':
    // ROD回転（y軸向かって、左方向）
    rod_q_d += 5.0 * M_PI / 180.0;
    break;
  case 's':
    // ROD回転（y軸向かって、右方向）
    rod_q_d -= 2.0 * M_PI / 180.0;
    break;
  case 'e':
    // RODリセット
    rod_q_d = 0.0;
    break;
  case 'r':
    // CANNONリセット
    {
      cannon_q_d[0] = 0.0;
      cannon_q_d[1] = 0.0;
    }
    break;
  case '[':
    // CANNON回転（水平、左右方向）
    cannon_q_d[0] += 0.101 * M_PI / 180.0;
    break;
  case ']':
    // CANNON回転（水平、左右方向）
    cannon_q_d[0] -= 0.101 * M_PI / 180.0;
    break;
  case '1':
    // CANNON回転（仰角、上）
    cannon_q_d[1] += 0.101 * M_PI / 180.0;
    break;
  case '2':
    // CANNON回転（仰角、下）
    cannon_q_d[1] -= 0.101 * M_PI / 180.0;
    break;
  case 'x':
    // CANNON発射
    {
      mytime = 0.0;
      dMatrix3 R2,R3,R4;
      dRFromAxisAndAngle(R2,0,0,1,cannon_q_d[0]);
      dRFromAxisAndAngle(R3,1,0,0,cannon_q_d[1] + 0.5 * M_PI);
      dMultiply0 (R4,R2,R3,3,3,3);
      dReal cpos[3] = {CANNON_X,CANNON_Y,CANNON_Z};
      for(int i=0; i<3; i++){
        cpos[i] += 3*R4[i*4+2];
      }
      dBodySetPosition(bullet.body,cpos[0],cpos[1],cpos[2]);
      dReal force = 500;//500;
      dBodySetLinearVel(bullet.body,force*R4[2],force*R4[6],force*R4[10]);
      dBodySetAngularVel(bullet.body,0,0,0);
      break;
    }
  default:
    break;
  }
}

// シミュレーションループ
static void simLoop(int pause) {
  control();
  dSpaceCollide(space,0,&nearCallback);  // 衝突検出計算
  dWorldStep(world,0.0001);                // 1ステップ進める
  dJointGroupEmpty(contactgroup);        // 衝突変数をリセット
  draw();  // ロボットの描画
}

static void start() {
  //static float xyz[3] = { 0.2,5.2,1.0};
  static float xyz[3] = { 1.0,1.5,1.0};
  static float hpr[3] = { 270,0,0};
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

/*** 描画関数の設定 ***/
void setDrawStuff() {
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.command = &command;      // command関数
  fn.path_to_textures = "textures"; // テクスチャ
}

int main (int argc, char *argv[]) {
  dInitODE();
  setDrawStuff();
  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world,0,0,0);
  dWorldSetERP(world,1.0);          // ERPの設定
  dWorldSetCFM(world,0.0);          // CFMの設定
  ground = dCreatePlane(space,0,0,1,0);
  create();
  dsSimulationLoop (argc,argv,500,600,&fn);
  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
