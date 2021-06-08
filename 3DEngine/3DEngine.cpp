#include "olcConsoleGameEngine.h"
#include <fstream>
#include <strstream>
#include <algorithm>

struct Vec3d {
    float x = 0;
    float y = 0;
    float z = 0;
    float w = 1;
};

struct Triangle {
    Vec3d p[3];
    wchar_t sym;
    short col;
};

struct Mesh {
    std::vector<Triangle> tris;
    
    bool loadFromObjectFile(std::string sFileName) {
        std::ifstream f(sFileName);
        if (!f.is_open())
            return false;
           
        //local cache of verts
        std::vector<Vec3d> verts;

        while (!f.eof()) {
            char line[128];
            f.getline(line, 128);
            Vec3d v;
            char junk;

            std::strstream s;
            s << line;

            if (line[0] == 'v') {
                s >> junk >> v.x >> v.y >> v.z;
                verts.push_back(v);
            }

            if(line[0]=='f'){
                int f[3];
                s >> junk >> f[0] >> f[1] >> f[2];
                tris.push_back({ verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1] });
                
             
            }
        }

        f.close();
        return true;
    }
};

struct Mat4x4 {
    float m[4][4] = { 0 };
};


class olcEngine3D : public olcConsoleGameEngine {
private: 
    Mesh meshCube;
    Mat4x4 matProj;

    Vec3d vCamera;   

    float fTheta;
    Vec3d Vector_Add(Vec3d& v1, Vec3d& v2)
    {
        return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
    }

    Vec3d Vector_Sub(Vec3d& v1, Vec3d& v2)
    {
        return { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };
    }

    Vec3d Vector_Mul(Vec3d& v1, float k)
    {
        return { v1.x * k, v1.y * k, v1.z * k };
    }

    Vec3d Vector_Div(Vec3d& v1, float k)
    {
        return { v1.x / k, v1.y / k, v1.z / k };
    }

    float Vector_DotProduct(Vec3d& v1, Vec3d& v2)
    {
        return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
    }

    float Vector_Length(Vec3d& v)
    {
        return sqrtf(Vector_DotProduct(v, v));
    }

    Vec3d Vector_Normalise(Vec3d& v)
    {
        float l = Vector_Length(v);
        return { v.x / l, v.y / l, v.z / l };
    }

    Vec3d Vector_CrossProduct(Vec3d& v1, Vec3d& v2)
    {
        Vec3d v;
        v.x = v1.y * v2.z - v1.z * v2.y;
        v.y = v1.z * v2.x - v1.x * v2.z;
        v.z = v1.x * v2.y - v1.y * v2.x;
        return v;
    }

    Vec3d Matrix_MultiplyVector(Mat4x4& m, Vec3d& i)
    {
        Vec3d v;
        v.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + i.w * m.m[3][0];
        v.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + i.w * m.m[3][1];
        v.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + i.w * m.m[3][2];
        v.w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + i.w * m.m[3][3];
        return v;
    }

    Mat4x4 Matrix_MakeIdentity()
    {
        Mat4x4 matrix;
        matrix.m[0][0] = 1.0f;
        matrix.m[1][1] = 1.0f;
        matrix.m[2][2] = 1.0f;
        matrix.m[3][3] = 1.0f;
        return matrix;
    }

    Mat4x4 Matrix_MakeRotationX(float fAngleRad)
    {
        Mat4x4 matrix;
        matrix.m[0][0] = 1.0f;
        matrix.m[1][1] = cosf(fAngleRad);
        matrix.m[1][2] = sinf(fAngleRad);
        matrix.m[2][1] = -sinf(fAngleRad);
        matrix.m[2][2] = cosf(fAngleRad);
        matrix.m[3][3] = 1.0f;
        return matrix;
    }

    Mat4x4 Matrix_MakeRotationY(float fAngleRad)
    {
        Mat4x4 matrix;
        matrix.m[0][0] = cosf(fAngleRad);
        matrix.m[0][2] = sinf(fAngleRad);
        matrix.m[2][0] = -sinf(fAngleRad);
        matrix.m[1][1] = 1.0f;
        matrix.m[2][2] = cosf(fAngleRad);
        matrix.m[3][3] = 1.0f;
        return matrix;
    }

    Mat4x4 Matrix_MakeRotationZ(float fAngleRad)
    {
        Mat4x4 matrix;
        matrix.m[0][0] = cosf(fAngleRad);
        matrix.m[0][1] = sinf(fAngleRad);
        matrix.m[1][0] = -sinf(fAngleRad);
        matrix.m[1][1] = cosf(fAngleRad);
        matrix.m[2][2] = 1.0f;
        matrix.m[3][3] = 1.0f;
        return matrix;
    }

    Mat4x4 Matrix_MakeTranslation(float x, float y, float z)
    {
        Mat4x4 matrix;
        matrix.m[0][0] = 1.0f;
        matrix.m[1][1] = 1.0f;
        matrix.m[2][2] = 1.0f;
        matrix.m[3][3] = 1.0f;
        matrix.m[3][0] = x;
        matrix.m[3][1] = y;
        matrix.m[3][2] = z;
        return matrix;
    }

    Mat4x4 Matrix_MakeProjection(float fFovDegrees, float fAspectRatio, float fNear, float fFar)
    {
        float fFovRad = 1.0f / tanf(fFovDegrees * 0.5f / 180.0f * 3.14159f);
        Mat4x4 matrix;
        matrix.m[0][0] = fAspectRatio * fFovRad;
        matrix.m[1][1] = fFovRad;
        matrix.m[2][2] = fFar / (fFar - fNear);
        matrix.m[3][2] = (-fFar * fNear) / (fFar - fNear);
        matrix.m[2][3] = 1.0f;
        matrix.m[3][3] = 0.0f;
        return matrix;
    }

    Mat4x4 Matrix_MultiplyMatrix(Mat4x4& m1, Mat4x4& m2)
    {
        Mat4x4 matrix;
        for (int c = 0; c < 4; c++)
            for (int r = 0; r < 4; r++)
                matrix.m[r][c] = m1.m[r][0] * m2.m[0][c] + m1.m[r][1] * m2.m[1][c] + m1.m[r][2] * m2.m[2][c] + m1.m[r][3] * m2.m[3][c];
        return matrix;
    }

    CHAR_INFO GetColour(float lum)
    {
        short bg_col, fg_col;
        wchar_t sym;
        int pixel_bw = (int)(13.0f * lum);
        switch (pixel_bw)
        {
        case 0: bg_col = BG_BLACK; fg_col = FG_BLACK; sym = PIXEL_SOLID; break;

        case 1: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_QUARTER; break;
        case 2: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_HALF; break;
        case 3: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_THREEQUARTERS; break;
        case 4: bg_col = BG_BLACK; fg_col = FG_DARK_GREY; sym = PIXEL_SOLID; break;

        case 5: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_QUARTER; break;
        case 6: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_HALF; break;
        case 7: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_THREEQUARTERS; break;
        case 8: bg_col = BG_DARK_GREY; fg_col = FG_GREY; sym = PIXEL_SOLID; break;

        case 9:  bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_QUARTER; break;
        case 10: bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_HALF; break;
        case 11: bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_THREEQUARTERS; break;
        case 12: bg_col = BG_GREY; fg_col = FG_WHITE; sym = PIXEL_SOLID; break;
        default:
            bg_col = BG_BLACK; fg_col = FG_BLACK; sym = PIXEL_SOLID;
        }

        CHAR_INFO c;
        c.Attributes = bg_col | fg_col;
        c.Char.UnicodeChar = sym;
        return c;
    }


public:
    olcEngine3D(){
        m_sAppName = L"3D Demo";
    }

public:
    bool OnUserCreate() override{

        meshCube.loadFromObjectFile("teapot.obj");

        //Projection Matrix
        matProj = Matrix_MakeProjection(90.f, (float)ScreenHeight()/(float)ScreenWidth(), 0.1f, 1000.0f);

        return true;
    }

public:
    bool OnUserUpdate(float fElapsedTime) override {
        Fill(0, 0, ScreenWidth(), ScreenHeight(), PIXEL_SOLID, FG_BLACK);

        Mat4x4 matRotX, matRotZ;
        fTheta += 1.0f * fElapsedTime;

        matRotZ = Matrix_MakeRotationZ(fTheta);
        matRotX = Matrix_MakeRotationX(fTheta);

        Mat4x4 matTrans;
        matTrans = Matrix_MakeTranslation(0.0f, 0.0f, 8.0f);

        Mat4x4 matWorld;
        matWorld = Matrix_MakeIdentity();
        matWorld = Matrix_MultiplyMatrix(matRotZ, matRotX);
        matWorld = Matrix_MultiplyMatrix(matWorld, matTrans);

        std::vector<Triangle> trianglesToRaster;

        //Draw Triangles 
        for (auto tri : meshCube.tris) {
            Triangle triProjected, triTransformed;
            triTransformed.p[0] = Matrix_MultiplyVector(matWorld, tri.p[0]);
            triTransformed.p[1] = Matrix_MultiplyVector(matWorld, tri.p[1]);
            triTransformed.p[2] = Matrix_MultiplyVector(matWorld, tri.p[2]);

            //Normal Calculations
            Vec3d normal, line1, line2;
            line1 = Vector_Sub(triTransformed.p[1], triTransformed.p[0]);
            line2 = Vector_Sub(triTransformed.p[2], triTransformed.p[0]);
            normal = Vector_CrossProduct(line1, line2);
            normal = Vector_Normalise(normal);
            
            Vec3d vCameraRay = Vector_Sub(triTransformed.p[0], vCamera);

            if (Vector_DotProduct(vCameraRay, normal) < 0.0f)
            {   
                //Illumination
                Vec3d light_direction = { 0.0f, 0.0f, -0.1f };
                light_direction = Vector_Normalise(light_direction);


                float dotProduct = Vector_DotProduct(light_direction, normal);
                
                CHAR_INFO c = GetColour(dotProduct);
                triTransformed.col = c.Attributes;
                triTransformed.sym = c.Char.UnicodeChar;

                //3D to 2D
                triProjected.p[0] = Matrix_MultiplyVector(matProj, triTransformed.p[0]);
                triProjected.p[1] = Matrix_MultiplyVector(matProj, triTransformed.p[1]);
                triProjected.p[2] = Matrix_MultiplyVector(matProj, triTransformed.p[2]);

                triProjected.col = triTransformed.col;
                triProjected.sym = triTransformed.sym;

                triProjected.p[0] = Vector_Div(triProjected.p[0], triProjected.p[0].w);
                triProjected.p[1] = Vector_Div(triProjected.p[1], triProjected.p[1].w);
                triProjected.p[2] = Vector_Div(triProjected.p[2], triProjected.p[2].w);

                //Scale into view
                Vec3d vOffsetView = { 1,1,0 };
                triProjected.p[0] = Vector_Add(triProjected.p[0], vOffsetView);
                triProjected.p[1] = Vector_Add(triProjected.p[1], vOffsetView);
                triProjected.p[2] = Vector_Add(triProjected.p[2], vOffsetView);

                triProjected.p[0].x *= 0.5f * (float)ScreenWidth();
                triProjected.p[0].y *= 0.5f * (float)ScreenWidth();
                triProjected.p[1].x *= 0.5f * (float)ScreenWidth();
                triProjected.p[1].y *= 0.5f * (float)ScreenWidth();
                triProjected.p[2].x *= 0.5f * (float)ScreenWidth();
                triProjected.p[2].y *= 0.5f * (float)ScreenWidth();

                trianglesToRaster.push_back(triProjected);
              
            }
        }
        sort(trianglesToRaster.begin(), trianglesToRaster.end(), 
            [](Triangle &t1, Triangle &t2){

                float z1 = (t1.p[0].z + t1.p[1].z + t1.p[2].z) / 3.0f;
                float z2 = (t2.p[0].z + t2.p[1].z + t2.p[2].z) / 3.0f;
                return z1 > z2;
                    
            });

        for (auto triProjected : trianglesToRaster) {
            FillTriangle(triProjected.p[0].x, triProjected.p[0].y,
                triProjected.p[1].x, triProjected.p[1].y,
                triProjected.p[2].x, triProjected.p[2].y,
                triProjected.sym, triProjected.col);

            /*DrawTriangle(triProjected.p[0].x, triProjected.p[0].y,
                triProjected.p[1].x, triProjected.p[1].y,
                triProjected.p[2].x, triProjected.p[2].y,
                PIXEL_SOLID, FG_BLACK);*/

        }
        return true;
    }
};

int main()
{   
    olcEngine3D engine;
    if (engine.ConstructConsole(256, 240, 2, 2));
        engine.Start();


}   

