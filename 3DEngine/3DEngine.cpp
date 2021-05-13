#include "olcConsoleGameEngine.h"

struct Vec3d {
    float x, y, z;
};

struct Triangle {
    Vec3d p[3];
};

struct Mesh {
    std::vector<Triangle> tris;
};

struct Mat4x4 {
    float m[4][4] = { 0 };
};


class olcEngine3D : public olcConsoleGameEngine {
private: 
    Mesh meshCube;
    Mat4x4 matProj;
    float fTheta;

    void multiplyMatrixVector(Vec3d &i, Vec3d &o, Mat4x4 &m) {
        o.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + m.m[3][0];
        o.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + m.m[3][1];
        o.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + m.m[3][2];
        float w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + m.m[3][3];

        if (w != 0) {
            o.x /= w;
            o.y /= w;
            o.z/= w;
        }
    }

public:
    olcEngine3D(){
        m_sAppName = L"3D Demo";
    }

public:
    bool OnUserCreate() override{
        meshCube.tris = {
            //SOUTH
            {0.0f, 0.0f, 0.0f,          0.0f, 1.0f, 0.0f,           1.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 0.0f,          1.0f, 1.0f, 0.0f,           1.0f, 0.0f, 0.0f}, 
            
            //EAST
            {1.0f, 0.0f, 0.0f,          1.0f, 1.0f, 0.0f,           1.0f, 1.0f, 1.0f},
            {1.0f, 0.0f, 0.0f,          1.0f, 1.0f, 1.0f,           1.0f, 0.0f, 1.0f},

            //NORTH
            {1.0f, 0.0f, 1.0f,          1.0f, 1.0f, 1.0f,           0.0f, 1.0f, 1.0f},
            {1.0f, 0.0f, 1.0f,          0.0f, 1.0f, 1.0f,           0.0f, 0.0f, 1.0f},

            //WEST
            {0.0f, 0.0f, 1.0f,          0.0f, 1.0f, 1.0f,           0.0f, 1.0f, 0.0f},
            {0.0f, 0.0f, 1.0f,          0.0f, 1.0f, 0.0f,           0.0f, 0.0f, 0.0f},

            //TOP
            {0.0f, 1.0f, 0.0f,          0.0f, 1.0f, 1.0f,           1.0f, 1.0f, 1.0f},
            {0.0f, 1.0f, 0.0f,          1.0f, 1.0f, 1.0f,           1.0f, 1.0f, 0.0f},

            //BOTTOM
            {1.0f, 0.0f, 1.0f,          0.0f, 0.0f, 1.0f,           0.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 1.0f,          0.0f, 0.0f, 0.0f,           1.0f, 0.0f, 0.0f},
            
        };

        //Projection Matrix
        float fNear = 0.1f;
        float fFar = 1000.0f;
        float fFov = 90.0f;
        float fAspectRatio = (float)ScreenHeight() / (float)ScreenWidth();
        float fFovRad = 1.0f / tanf(fFov * 0.5f / 180.0f * 3.14159f);

        matProj.m[0][0] = fAspectRatio * fFovRad;
        matProj.m[1][1] = fFovRad;
        matProj.m[2][2] = fFar / (fFar - fNear);
        matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
        matProj.m[2][3] = 1.0f;
        matProj.m[3][3] = 0.0f;

        return true;
    }

public:
    bool OnUserUpdate(float fElapsedTime) override {
        Fill(0, 0, ScreenWidth(), ScreenHeight(), PIXEL_SOLID, FG_BLACK);

        Mat4x4 matRotX, matRotZ;
        fTheta += 1.0f * fElapsedTime;

        //Rotation X
        matRotX.m[0][0] = 1;
        matRotX.m[1][1] = cosf(fTheta * 0.5);
        matRotX.m[1][2] = sinf(fTheta * 0.5);
        matRotX.m[2][1] = -sinf(fTheta * 0.5);
        matRotX.m[2][2] = cosf(fTheta * 0.5);
        matRotX.m[3][3] = 1;

        //Rotation Z
        matRotZ.m[0][0] = cosf(fTheta);
        matRotZ.m[0][1] = sinf(fTheta);
        matRotZ.m[1][0] = -sinf(fTheta);
        matRotZ.m[1][1] = cosf(fTheta);
        matRotZ.m[2][2] = 1;
        matRotZ.m[3][3] = 1;

        //Draw Triangles
        for (auto tri : meshCube.tris) {
            Triangle triProjected, triTranslated, triRotatedX, triRotatedXZ;

            //Rotating
            multiplyMatrixVector(tri.p[0], triRotatedX.p[0], matRotX);
            multiplyMatrixVector(tri.p[1], triRotatedX.p[1], matRotX);
            multiplyMatrixVector(tri.p[2], triRotatedX.p[2], matRotX); 
            
            multiplyMatrixVector(triRotatedX.p[0], triRotatedXZ.p[0], matRotZ);
            multiplyMatrixVector(triRotatedX.p[1], triRotatedXZ.p[1], matRotZ);
            multiplyMatrixVector(triRotatedX.p[2], triRotatedXZ.p[2], matRotZ);

            //Translating
            triTranslated = triRotatedXZ;
            triTranslated.p[0].z = triRotatedXZ.p[0].z + 3.0f;
            triTranslated.p[1].z = triRotatedXZ.p[1].z + 3.0f;
            triTranslated.p[2].z = triRotatedXZ.p[2].z + 3.0f;

            multiplyMatrixVector(triTranslated.p[0], triProjected.p[0], matProj);
            multiplyMatrixVector(triTranslated.p[1], triProjected.p[1], matProj);
            multiplyMatrixVector(triTranslated.p[2], triProjected.p[2], matProj);

            //Scale into view
            triProjected.p[0].x += 1.0f; triProjected.p[0].y += 1.0f;
            triProjected.p[1].x += 1.0f; triProjected.p[1].y += 1.0f;
            triProjected.p[2].x += 1.0f; triProjected.p[2].y += 1.0f;

            triProjected.p[0].x *= 0.5f * (float)ScreenWidth();
            triProjected.p[0].y *= 0.5f * (float)ScreenWidth();
            triProjected.p[1].x *= 0.5f * (float)ScreenWidth();
            triProjected.p[1].y *= 0.5f * (float)ScreenWidth();
            triProjected.p[2].x *= 0.5f * (float)ScreenWidth();
            triProjected.p[2].y *= 0.5f * (float)ScreenWidth();


            DrawTriangle(triProjected.p[0].x, triProjected.p[0].y,
                triProjected.p[1].x, triProjected.p[1].y,
                triProjected.p[2].x, triProjected.p[2].y,
                PIXEL_SOLID, FG_WHITE);
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

