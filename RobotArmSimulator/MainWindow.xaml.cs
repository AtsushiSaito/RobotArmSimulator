


using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using SharpGL;//SharpGLのインストールをする必要あり
using MathNet.Numerics.LinearAlgebra.Double;


namespace RobotArmSimulator
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        //以下変数の定義

        const double PI = 3.141592653589;

        int J1_ULMT = 150; //第1関節角度リミット 正方向
        int J1_LLMT = -150; //第1関節角度リミット 負方向
        int J2_ULMT = 120; //第2関節角度リミット 正方向
        int J2_LLMT = -60; //第2関節角度リミット 負方向
        int J3_ULMT = 120; //第3関節角度リミット 正方向
        int J3_LLMT = -110; //第3関節角度リミット 負方向
        int J4_ULMT = 90; //第4関節角度リミット 正方向
        int J4_LLMT = -90; //第4関節角度リミット 負方向
        int J5_ULMT = 90; //第5関節角度リミット 正方向
        int J5_LLMT = -90; //第5関節角度リミット 負方向
        float h0 = 300, h1 = 250, h2 = 160, h3 = 72;
        double X_Pos, Y_Pos;
        double[] Theta = new double[10];//アーム角度用
        double[] JointAngle = new double[10];//アーム角度用
        double T;
        double[] theta = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };


        double[,] inv = { { 0, 0, 0 }, { 0, 0, 0, }, { 0, 0, 0 } };
        double[,] a = { { 0, 0, 0 }, { 0, 0, 0, }, { 0, 0, 0 } };
        double[,] b = { { 0, 0, 0 }, { 0, 0, 0, }, { 0, 0, 0 } };
        double[,] c = { { 0, 0, 0 }, { 0, 0, 0, }, { 0, 0, 0 } };
        double[,] c_old = { { 0, 0, 0 }, { 0, 0, 0, }, { 0, 0, 0 } };


        public MainWindow()//ウィンドウ生成時の実行内容
        {
            InitializeComponent();

            X_Slider.Minimum = 0;
            X_Slider.Maximum = 450;
            Y_Slider.Minimum = -250;
            Y_Slider.Maximum = 250;
            X_Slider.Value = 350;
            Y_Slider.Value = 100;
            for (int i = 0; i < 8; i++) JointAngle[i] = 0;//角度初期化
            J0_TextBox.Text = JointAngle[0].ToString();
            J1_TextBox.Text = JointAngle[1].ToString();
            J2_TextBox.Text = JointAngle[2].ToString();
            J3_TextBox.Text = JointAngle[3].ToString();
            J4_TextBox.Text = JointAngle[4].ToString();
            //this.DataContext = new { J0_Angle_Data };//int型のオブジェクトをインスタンス化してXにX++したものを渡す？

        }
        //以下OpenGLの処理内容
        private void OpenGLControl_OpenGLDraw(object sender, SharpGL.SceneGraph.OpenGLEventArgs args)//描画内容を書く
        {
            if ((bool)Forward_Kinematics_Check.IsChecked)
            {
                this.J0_Slider.IsEnabled = true;
                this.J1_Slider.IsEnabled = true;
                this.J2_Slider.IsEnabled = true;
                this.J3_Slider.IsEnabled = true;
                this.J4_Slider.IsEnabled = true;
                JointAngle[0] = J0_Slider.Value;
                JointAngle[1] = J1_Slider.Value;
                JointAngle[2] = J2_Slider.Value;
                JointAngle[3] = J3_Slider.Value;
                JointAngle[4] = J4_Slider.Value;
                J0_TextBox.Text = JointAngle[0].ToString("F2");
                J1_TextBox.Text = JointAngle[1].ToString("F2");
                J2_TextBox.Text = JointAngle[2].ToString("F2");
                J3_TextBox.Text = JointAngle[3].ToString("F2");
                J4_TextBox.Text = JointAngle[4].ToString("F2");
            }
            else
            {
                this.J0_Slider.IsEnabled = false;
                this.J1_Slider.IsEnabled = false;
                this.J2_Slider.IsEnabled = false;
                this.J3_Slider.IsEnabled = false;
                this.J4_Slider.IsEnabled = false;
            }
            if ((bool)Inverse_Kinematics_Check.IsChecked)
            {



                X_Pos = X_Slider.Value;
                Y_Pos = Y_Slider.Value;
                X_TextBox.Text = X_Pos.ToString("F2");
                Y_TextBox.Text = Y_Pos.ToString("F2");
                int i, j, k, e = 3;
                double[,] Jacob = {
                { -h1*Math.Sin(theta[1])-h2*Math.Sin(theta[1]+theta[2])-(h3)*Math.Sin(theta[1]+theta[2]+theta[3]),
                  -h2*Math.Sin(theta[1]+theta[2])-(h3)*Math.Sin(theta[1]+theta[2]+theta[3]),
                  -(h3)*Math.Sin(theta[1]+theta[2]+theta[3]) },
                { h1*Math.Cos(theta[1])+h2*Math.Cos(theta[1]+theta[2])+(h3)*Math.Cos(theta[1]+theta[2]+theta[3]),
                  h2*Math.Cos(theta[1]+theta[2])+(h3)*Math.Cos(theta[1]+theta[2]+theta[3]),
                 (h3)*Math.Cos(theta[1]+theta[2]+theta[3])},
                { -1, -1, -1 }};//ヤコビを定義

                double[] Func = {
                  h1*Math.Cos(theta[1])+h2*Math.Cos(theta[1]+theta[2])+(h3)*Math.Cos(theta[1]+theta[2]+theta[3])-X_Pos,
                  h1*Math.Sin(theta[1])+h2*Math.Sin(theta[1]+theta[2])+(h3)*Math.Sin(theta[1]+theta[2]+theta[3])-Y_Pos,
                  -(theta[1]+theta[2]+theta[3])-T };

                for (i = 0; i < e; i++)
                {
                    for (j = 0; j < e; j++)
                    {
                        a[i, j] = Jacob[i, j];
                        b[i, j] = Func[i];
                    }
                }
                for (i = 0; i < e; i++)
                {
                    for (j = 0; j < e; j++)
                    {
                        inv[i, j] = 0.0;
                        if (i == j)
                        {
                            inv[i, j] = 1.0;//対角を1
                        }
                    }
                }
                for (i = 0; i < e; i++)
                {//要素数だけループ
                    double buf = a[i, i];//元の対角成分をバッファをコピー
                    for (j = 0; j < e; j++)
                    {//行要素分だけループ
                        a[i, j] *= 1 / buf;//バッファで行成分をすべて割る
                        inv[i, j] *= 1 / buf;//単位行列の行成分を割る。
                    }
                    for (j = 0; j < e; j++)
                    {
                        if (i != j)
                        {//対角成分ではないとき
                            buf = a[j, i];//i列成分をバッファにコピー
                            for (k = 0; k < e; k++)
                            {
                                a[j, k] -= a[i, k] * buf;
                                inv[j, k] -= inv[i, k] * buf;
                            }
                        }
                    }
                }
                for (i = 0; i < e; i++)
                {
                    for (j = 0; j < e; j++)
                    {
                        c[i, j] = c_old[i, j];
                        for (k = 0; k < e; k++)
                        {
                            c[i, j] += -(inv[i, k] * b[k, j]);
                        }
                        theta[i + 1] = c[i, 0];
                    }
                }
                for (i = 0; i < e; i++)
                {
                    for (j = 0; j < e; j++)
                    {
                        c_old[i, j] = c[i, j];
                        c[i, j] = 0;
                    }
                }

                Theta[1] = (theta[1] * 180 / PI) % 360;
                Theta[2] = (theta[2] * 180 / PI) % 360;
                Theta[3] = (theta[3] * 180 / PI) % 360;

                if (Theta[1] > 180)
                {
                    Theta[1] -= 360;
                }
                if (Theta[1] < -180)
                {
                    Theta[1] += 360;
                }

                if (Theta[2] > 180)
                {
                    Theta[2] -= 360;
                }
                else if (Theta[2] < -180)
                {
                    Theta[2] += 360;
                }

                if (Theta[3] > 180)
                {
                    Theta[3] -= 360;
                }
                else if (Theta[3] < -180)
                {
                    Theta[3] += 360;
                }
                /********************************/
                if (Theta[1] > J2_ULMT)
                {
                    Theta[1] = J2_ULMT;
                }
                if (Theta[1] < J2_LLMT)
                {
                    Theta[1] = J2_LLMT;
                }
                if (Theta[2] > J3_ULMT)
                {
                    Theta[2] = J3_ULMT;
                }
                if (Theta[2] < J3_LLMT)
                {
                    Theta[2] = J3_LLMT;
                }
                if (Theta[3] > J4_ULMT)
                {
                    Theta[3] = J4_ULMT;
                }
                if (Theta[3] < J4_LLMT)
                {
                    Theta[3] = J4_LLMT;
                }

                if ((Theta[1] < 0.1 && Theta[1] > -0.1) && (Theta[2] < 0.1 && Theta[2] > -0.1) && (Theta[3] < 0.1 && Theta[3] > -0.1))
                {
                    Theta[1] = 0.1;
                    Theta[2] = 0.1;
                    Theta[3] = 0.1;
                }
                JointAngle[1] = Theta[1];
                JointAngle[2] = Theta[2];
                JointAngle[3] = Theta[3];


                /*double K = Math.Sqrt(Math.Pow(Math.Pow(X_Pos, 2) + Math.Pow(Y_Pos, 2) + Math.Pow(h1, 2) + Math.Pow(h2, 2), 2) - (2 * (Math.Pow(Math.Pow(X_Pos, 2) + Math.Pow(Y_Pos, 2), 2) + Math.Pow(h1, 4) + Math.Pow(h2, 4))));
                Theta[0] = Math.Atan2(Y_Pos, X_Pos) - Math.Atan2(K, (X_Pos * X_Pos) + (Y_Pos * Y_Pos) + (h2 * 2) - (h2 * 2));
                Theta[1] = Math.Atan2(K, (X_Pos * X_Pos) + (Y_Pos * Y_Pos) - Math.Pow(h1, 2) - Math.Pow(h2, 2));
                Theta[2] = 0 - (Theta[0] + Theta[1]);
                JointAngle[1] = Theta[0] * 180 / PI;
                JointAngle[2] = Theta[1] * 180 / PI;
                JointAngle[3] = Theta[2] * 180 / PI;
                 */


                J0_TextBox.Text = JointAngle[0].ToString("F2");
                J1_TextBox.Text = JointAngle[1].ToString("F2");
                J2_TextBox.Text = JointAngle[2].ToString("F2");
                J3_TextBox.Text = JointAngle[3].ToString("F2");
                J4_TextBox.Text = JointAngle[4].ToString("F2");


            }
            else
            {

            }


            OpenGL gl = args.OpenGL;
            gl.Clear(OpenGL.GL_COLOR_BUFFER_BIT | OpenGL.GL_DEPTH_BUFFER_BIT);
            gl.LoadIdentity();
            gl.Enable(OpenGL.GL_LINE_SMOOTH);
            gl.Hint(OpenGL.GL_LINE_SMOOTH_HINT, OpenGL.GL_NICEST);
            gl.Enable(OpenGL.GL_BLEND);
            gl.BlendFunc(OpenGL.GL_SRC_ALPHA, OpenGL.GL_ONE_MINUS_SRC_ALPHA);
            gl.LookAt(900, 900, 900, 0, 200, 0, 0, 1, 0);

            gl.Rotate(-90, 1.0f, 0.0f, 0.0f);//Z軸を上方向にするための回転

            double axis = 100, axis_R = 150; ;

            gl.LineWidth(3);
            /**********座標軸**********/
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 0.0f, 0.0f);
            gl.Vertex(axis_R, 0.0f, 0.0f);//X軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 1.0f, 0.0f);
            gl.Vertex(0.0f, axis_R, 0.0f);//Y軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 0.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, axis_R);//Z軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.End();
            /**************************/
            gl.Rotate(JointAngle[0], 0.0f, 0.0f, 1.0f);
            gl.Translate(0, 0, h0);//平行移動(diの0)
            /**********アームh0**********/
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 1.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Vertex(0.0f, 0.0f, -h0);
            /**********座標軸**********/
            gl.Color(1.0f, 0.0f, 0.0f);
            gl.Vertex(axis, 0.0f, 0.0f);//X軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 1.0f, 0.0f);
            gl.Vertex(0.0f, axis, 0.0f);//Y軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 0.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, axis);//Z軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.End();
            /*************************/
            gl.Rotate(90, 1.0, 0.0, 0.0);//x軸回転(αの２)
            /**********座標軸**********/
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 0.0f, 0.0f);
            gl.Vertex(axis, 0.0f, 0.0f);//X軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 1.0f, 0.0f);
            gl.Vertex(0.0f, axis, 0.0f);//Y軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 0.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, axis);//Z軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.End();
            /**************************/
            gl.Rotate(JointAngle[1], 0.0f, 0.0f, 1.0f);
            gl.Translate(h1, 0, 0);//平行移動(Aiの3)
            /**********アームh1**********/
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 1.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Vertex(-h1, 0.0f, 0.0f);
            /**********座標軸**********/
            gl.Color(1.0f, 0.0f, 0.0f);
            gl.Vertex(axis, 0.0f, 0.0f);//X軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 1.0f, 0.0f);
            gl.Vertex(0.0f, axis, 0.0f);//Y軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 0.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, axis);//Z軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.End();
            /*************************/
            gl.Rotate(JointAngle[2], 0.0f, 0.0f, 1.0f);
            gl.Translate(h2, 0, 0);//平行移動(Aiの4)
            /**********アームh2**********/
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 1.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Vertex(-h2, 0.0f, 0.0f);
            /**********座標軸**********/
            gl.Color(1.0f, 0.0f, 0.0f);
            gl.Vertex(axis, 0.0f, 0.0f);//X軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 1.0f, 0.0f);
            gl.Vertex(0.0f, axis, 0.0f);//Y軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 0.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, axis);//Z軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.End();
            /*************************/
            gl.Rotate(JointAngle[3], 0.0f, 0.0f, 1.0f);
            gl.Rotate(90, 1.0f, 0.0f, 0.0f);
            /**********座標軸**********/
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 0.0f, 0.0f);
            gl.Vertex(axis, 0.0f, 0.0f);//X軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 1.0f, 0.0f);
            gl.Vertex(0.0f, axis, 0.0f);//Y軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 0.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, axis);//Z軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.End();
            /*************************/
            gl.Rotate(JointAngle[4], 0.0f, 0.0f, 1.0f);
            gl.Translate(0, 0, h3);//平行移動(diのHand)
            /**********アームh3**********/
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 1.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Vertex(0.0f, 0.0f, -h3);
            /**********座標軸**********/
            gl.Color(1.0f, 0.0f, 0.0f);
            gl.Vertex(axis, 0.0f, 0.0f);//X軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 1.0f, 0.0f);
            gl.Vertex(0.0f, axis, 0.0f);//Y軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.Color(0.0f, 0.0f, 1.0f);
            gl.Vertex(0.0f, 0.0f, axis);//Z軸
            gl.Vertex(0.0f, 0.0f, 0.0f);
            gl.End();
            /**********手先**********/
            double Hand_Width = h3 / 3;
            gl.Begin(OpenGL.GL_LINES);
            gl.Color(1.0f, 1.0f, 1.0f);
            gl.Vertex(-Hand_Width, 0.0f, -Hand_Width);
            gl.Vertex(Hand_Width, 0.0f, -Hand_Width);
            gl.Vertex(Hand_Width, 0.0f, -Hand_Width);
            gl.Vertex(Hand_Width, 0.0f, Hand_Width);
            gl.Vertex(-Hand_Width, 0.0f, -Hand_Width);
            gl.Vertex(-Hand_Width, 0.0f, Hand_Width);
            gl.End();
        }
        private void OpenGLControl_OpenGLInitialized(object sender, SharpGL.SceneGraph.OpenGLEventArgs args)
        {
            args.OpenGL.Enable(OpenGL.GL_DEPTH_TEST);
        }
        private void OpenGLControl_Resized(object sender, SharpGL.SceneGraph.OpenGLEventArgs args)
        {
            OpenGL gl = args.OpenGL;
            gl.MatrixMode(OpenGL.GL_PROJECTION);
            gl.LoadIdentity();
            gl.ClearColor(0.0f, 0.0f, 0.0f, 1.0f);//カラーバッファのクリアの色
            gl.Perspective(45.0f, (float)gl.RenderContextProvider.Width /
                (float)gl.RenderContextProvider.Height,
                0.1f, 2000);
            gl.MatrixMode(OpenGL.GL_MODELVIEW);
        }


    }
}
