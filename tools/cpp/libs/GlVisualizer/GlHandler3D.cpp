/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2013 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <GlHandler3D.h>
#include <pangolin/handler/handler.h>
#include <pangolin/display/display_internal.h>
#include <pangolin/display/view.h>

namespace pangolin
{

// Pointer to context defined in display.cpp
    extern __thread PangolinGl* context;

    GlHandler3D::GlHandler3D(OpenGlRenderState& cam_state, AxisDirection enforce_up, float trans_scale, float zoom_fraction)
            : cam_state(&cam_state), enforce_up(enforce_up), tf(trans_scale), zf(zoom_fraction), cameraspec(CameraSpecOpenGl), last_z(0.8)
    {
        SetZero<3,1>(rot_center);
    }

    void GlHandler3D::Keyboard(View&, unsigned char key, int x, int y, bool pressed)
    {
        /*std::cout << "key = " << int(key) << std::endl;
        std::cout << "x = " << x << std::endl;
        std::cout << "y = " << y << std::endl;
        std::cout << "pressed = " << pressed << std::endl;*/

        if (pressed) return; // only perform action on key release

        for (auto& h : keyboardButtonHandlers) {
            if (h.group == Characters) {
                if (h.keyboardCondition == Ctrl && funcKeyState & pangolin::KeyModifierCtrl && key >= 1 && key <= 26) { // CTRL + (SHIFT) + A-Z
                    if (h.handler)
                        h.handler(key + (97-1));
                }
                else if (h.keyboardCondition == Shift && funcKeyState & pangolin::KeyModifierShift && key >= 65 && key <= 90) { // SHIFT + A-Z
                    if (h.handler)
                        h.handler(key + (97-65));
                }
                else if (h.keyboardCondition == None && key >= 97 && key <= 122) { // A-Z
                    if (h.handler)
                        h.handler(key);
                }
            }
            else if (h.group == Numbers) {
                if (key >= 48 && key <= 57) { // 0-9
                    if (h.handler)
                        h.handler(key);
                }
            }
        }
    }

    bool GlHandler3D::ValidWinDepth(GLprecision depth)
    {
        return depth != 1;
    }

    void GlHandler3D::PixelUnproject( View& view, GLprecision winx, GLprecision winy, GLprecision winz, GLprecision Pc[3])
    {
        const GLint viewport[4] = {view.v.l,view.v.b,view.v.w,view.v.h};
        const pangolin::OpenGlMatrix proj = cam_state->GetProjectionMatrix();
        glUnProject(winx, winy, winz, Identity4d, proj.m, viewport, &Pc[0], &Pc[1], &Pc[2]);
    }

    void GlHandler3D::GetPosNormal(pangolin::View& view, int winx, int winy, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision nw[3], GLprecision default_z)
    {
        // TODO: Get to work on android

        const int zl = (hwin*2+1);
        const int zsize = zl*zl;
        GLfloat zs[zsize];

#ifndef HAVE_GLES
        glReadBuffer(GL_FRONT);
        glReadPixels(winx-hwin,winy-hwin,zl,zl,GL_DEPTH_COMPONENT,GL_FLOAT,zs);
#else
        std::fill(zs,zs+zsize, 1);
#endif
        GLfloat mindepth = *(std::min_element(zs,zs+zsize));

        if(mindepth == 1) mindepth = (GLfloat)default_z;

        p[0] = winx; p[1] = winy; p[2] = mindepth;
        PixelUnproject(view, winx, winy, mindepth, Pc);

        const pangolin::OpenGlMatrix mv = cam_state->GetModelViewMatrix();

        GLprecision T_wc[3*4];
        LieSE3from4x4(T_wc, mv.Inverse().m );
        LieApplySE3vec(Pw, T_wc, Pc);

        // Neighboring points in camera coordinates
        GLprecision Pl[3]; GLprecision Pr[3]; GLprecision Pb[3]; GLprecision Pt[3];
        PixelUnproject(view, winx-hwin, winy, zs[hwin*zl + 0],    Pl );
        PixelUnproject(view, winx+hwin, winy, zs[hwin*zl + zl-1], Pr );
        PixelUnproject(view, winx, winy-hwin, zs[hwin+1],         Pb );
        PixelUnproject(view, winx, winy+hwin, zs[zsize-(hwin+1)], Pt );

        // n = ((Pr-Pl).cross(Pt-Pb)).normalized();
        GLprecision PrmPl[3]; GLprecision PtmPb[3];
        MatSub<3,1>(PrmPl,Pr,Pl);
        MatSub<3,1>(PtmPb,Pt,Pb);

        GLprecision nc[3];
        CrossProduct(nc, PrmPl, PtmPb);
        Normalise<3>(nc);

        // T_wc is col major, so the rotation component is first.
        LieApplySO3(nw,T_wc,nc);
    }


    void GlHandler3D::GetPos(pangolin::View& view, int winx, int winy, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision default_z)
    {
        // TODO: Get to work on android

        const int hwin_small = 1;
        const int zl = (hwin_small*2+1);
        const int zsize = zl*zl;
        GLfloat zs[zsize];

#ifndef HAVE_GLES
        glReadBuffer(GL_FRONT);
        glReadPixels(winx-hwin_small,winy-hwin_small,zl,zl,GL_DEPTH_COMPONENT,GL_FLOAT,zs);
#else
        std::fill(zs,zs+zsize, 1);
#endif
        GLfloat mindepth = *(std::min_element(zs,zs+zsize));
        GLfloat meandepth = 0;

        int count = 0;
        for (int i = 0; i < zsize; i++) {
            if (zs[i] < 1) {
                meandepth += zs[i]*zs[i];
                count++;
            }
        }

        if (count > 0) {
            meandepth /= count;
            meandepth = sqrtf(meandepth);
        }
        else meandepth = 1;

        if(mindepth == 1) mindepth = (GLfloat)default_z;

        p[0] = winx; p[1] = winy; p[2] = meandepth;
        PixelUnproject(view, winx, winy, meandepth, Pc);

        const pangolin::OpenGlMatrix mv = cam_state->GetModelViewMatrix();

        GLprecision T_wc[3*4];
        LieSE3from4x4(T_wc, mv.Inverse().m );
        LieApplySE3vec(Pw, T_wc, Pc);
    }

    // Mouse click function
    // 'button' holds the current mouse button firing/triggering this function, hence only one button value
    // 'pressed' indicates whether 'button' was pressed or released
    // 'button_state' holds all the pressed buttons including keyboard keys
    void GlHandler3D::Mouse(View& display, MouseButton button, int x, int y, bool pressed, int button_state)
    {
        // mouse down
        last_pos[0] = (float)x;
        last_pos[1] = (float)y;

        GLprecision T_nc[3*4];
        LieSetIdentity(T_nc);

        funcKeyState = 0;

        if (pressed)
        {
            Eigen::Vector3d mouse3Dposition = getMouse3Dposition();
            for (auto& h : mouseButtonHandlers) {
                if (button == int(h.button)) {
                    if (!h.keyboardCondition || button_state & h.keyboardCondition) {
                        if (h.handler)
                            h.handler(mouse3Dposition);
                    }
                }
            }

            // Call handler at mouse release event
            if (!pressed && (button_state & KeyModifierCtrl) && (button == MouseButtonLeft || button == MouseButtonRight)) {
                // point selection by drawing
                if (highlightSingleHandler) {
                    if (button_state == MouseButtonLeft)
                        highlightSingleHandler(getMouse3Dposition(), true, false);
                    else
                        highlightSingleHandler(getMouse3Dposition(), false, true);
                }
            }
        }

        /*std::cout << "button = " << button << std::endl;
        std::cout << "pressed = " << pressed << std::endl;
        std::cout << "button_state = " << button_state << std::endl;
        std::cout << std::endl;*/

        /*if (button_state & KeyModifierCtrl) {
            if (pressed && button == MouseButtonLeft) {
                std::cout << "Left button pressed" << std::endl;
                if (leftHandler)
                    leftHandler(Pw2[0], Pw2[1], Pw2[2]);
            }
            else if (pressed && button == MouseButtonRight) {
                std::cout << "Right button pressed" << std::endl;
                if (rightHandler)
                    rightHandler(Pw2[0], Pw2[1], Pw2[2]);
            }
        } else {*/
            if (pressed) {
                GetPosNormal(display, x, y, p, Pw, Pc, n, last_z);
                if (ValidWinDepth(p[2])) {
                    last_z = p[2];
                    std::copy(Pc, Pc + 3, rot_center);
                }

                if (button == MouseWheelUp || button == MouseWheelDown) {
                    LieSetIdentity(T_nc);
                    const GLprecision t[3] = {0, 0, (button == MouseWheelUp ? 1 : -1) * 100 * tf};
                    LieSetTranslation<>(T_nc, t);
                    if (!(button_state & MouseButtonRight) &&
                        !(rot_center[0] == 0 && rot_center[1] == 0 && rot_center[2] == 0)) {
                        LieSetTranslation<>(T_nc, rot_center);
                        const GLprecision s = (button == MouseWheelUp ? -1.0 : 1.0) * zf;
                        MatMul<3, 1>(T_nc + (3 * 3), s);
                    }
                    OpenGlMatrix &spec = cam_state->GetModelViewMatrix();
                    LieMul4x4bySE3<>(spec.m, T_nc, spec.m);
                }
            }
        //}
        funcKeyState = button_state;
    }

    void GlHandler3D::MouseMotion(View& display, int x, int y, int button_state)
    {
        const GLprecision rf = 0.01;
        const float delta[2] = { (float)x - last_pos[0], (float)y - last_pos[1] };
        const float mag = delta[0]*delta[0] + delta[1]*delta[1];

        if ((button_state & KeyModifierCtrl) && (button_state == MouseButtonLeft || button_state == MouseButtonRight)) {
            // point selection by drawing
            if (highlightDrawingHandler) {
                if (button_state == MouseButtonLeft)
                    highlightDrawingHandler(getMouse3Dposition(), true, false);
                else
                    highlightDrawingHandler(getMouse3Dposition(), false, true);
            }
        }

        if((button_state & KeyModifierCtrl) && (button_state & KeyModifierShift))
        {
            GLprecision T_nc[3 * 4];
            LieSetIdentity(T_nc);

            GetPosNormal(display, x, y, p, Pw, Pc, n, last_z);
            if(ValidWinDepth(p[2]))
            {
                last_z = p[2];
                std::copy(Pc, Pc + 3, rot_center);
            }
        }

        funcKeyState = button_state;

        // TODO: convert delta to degrees based of fov
        // TODO: make transformation with respect to cam spec
        if( mag < 50.0f*50.0f && !(button_state & KeyModifierCtrl))
        {
            OpenGlMatrix& mv = cam_state->GetModelViewMatrix();
            const GLprecision* up = AxisDirectionVector[enforce_up];
            GLprecision T_nc[3*4];
            LieSetIdentity(T_nc);
            bool rotation_changed = false;

            if( button_state == MouseButtonMiddle )
            {
                // Middle Drag: Rotate around view

                // Try to correct for different coordinate conventions.
                GLprecision aboutx = -rf * delta[1];
                GLprecision abouty = rf * delta[0];
                OpenGlMatrix& pm = cam_state->GetProjectionMatrix();
                abouty *= -pm.m[2 * 4 + 3];

                Rotation<>(T_nc, aboutx, abouty, (GLprecision)0.0);
            }else if( button_state == MouseButtonLeft )
            {
                // Left Drag: in plane translate
                if( ValidWinDepth(last_z) )
                {
                    GLprecision np[3];
                    PixelUnproject(display, x, y, last_z, np);
                    const GLprecision t[] = { np[0] - rot_center[0], np[1] - rot_center[1], 0};
                    LieSetTranslation<>(T_nc,t);
                    std::copy(np,np+3,rot_center);
                }else{
                    const GLprecision t[] = { -10*delta[0]*tf, 10*delta[1]*tf, 0};
                    LieSetTranslation<>(T_nc,t);
                }
            }else if( button_state == (MouseButtonLeft | MouseButtonRight) )
            {
                // Left and Right Drag: in plane rotate about object
                //        Rotation<>(T_nc,0.0,0.0, delta[0]*0.01);

                GLprecision T_2c[3*4];
                Rotation<>(T_2c, (GLprecision)0.0, (GLprecision)0.0, delta[0]*rf);
                GLprecision mrotc[3];
                MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
                LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
                GLprecision T_n2[3*4];
                LieSetIdentity<>(T_n2);
                LieSetTranslation<>(T_n2,rot_center);
                LieMulSE3(T_nc, T_n2, T_2c );
                rotation_changed = true;
            }else if( button_state == MouseButtonRight)
            {
                GLprecision aboutx = -rf * delta[1];
                GLprecision abouty = -rf * delta[0];

                // Try to correct for different coordinate conventions.
                if(cam_state->GetProjectionMatrix().m[2*4+3] <= 0) {
                    abouty *= -1;
                }

                if(enforce_up) {
                    // Special case if view direction is parallel to up vector
                    const GLprecision updotz = mv.m[2]*up[0] + mv.m[6]*up[1] + mv.m[10]*up[2];
                    if(updotz > 0.98) aboutx = std::min(aboutx, (GLprecision)0.0);
                    if(updotz <-0.98) aboutx = std::max(aboutx, (GLprecision)0.0);
                    // Module rotation around y so we don't spin too fast!
                    abouty *= (1-0.6*fabs(updotz));
                }

                // Right Drag: object centric rotation
                GLprecision T_2c[3*4];
                Rotation<>(T_2c, aboutx, abouty, (GLprecision)0.0);
                GLprecision mrotc[3];
                MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
                LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
                GLprecision T_n2[3*4];
                LieSetIdentity<>(T_n2);
                LieSetTranslation<>(T_n2,rot_center);
                LieMulSE3(T_nc, T_n2, T_2c );
                rotation_changed = true;
            }

            LieMul4x4bySE3<>(mv.m,T_nc,mv.m);

            if(enforce_up != AxisNone && rotation_changed) {
                EnforceUpT_cw(mv.m, up);
            }
        }

        last_pos[0] = (float)x;
        last_pos[1] = (float)y;
    }

    void GlHandler3D::PassiveMouseMotion(View& display, int x, int y, int button_state)
    {
        GLprecision _p[3];
        GLprecision _Pw[3];
        GLprecision _Pc[3];

        //if (button_state & KeyModifierCtrl) {
            GetPos(display, x, y, _p, _Pw, _Pc, 1);
            if (_p[2] < 1) {
                Pw2[0] = _Pw[0];
                Pw2[1] = _Pw[1];
                Pw2[2] = _Pw[2];
            }
            funcKeyState = button_state;
        //}
    }

    void GlHandler3D::Special(View& display, InputSpecial inType, float x, float y, float p1, float p2, float /*p3*/, float /*p4*/, int button_state)
    {
        if( !(inType == InputSpecialScroll || inType == InputSpecialRotate) )
            return;

        // mouse down
        last_pos[0] = x;
        last_pos[1] = y;

        GLprecision T_nc[3*4];
        LieSetIdentity(T_nc);

        GetPosNormal(display, (int)x, (int)y, p, Pw, Pc, n, last_z);
        if(p[2] < 1.0) {
            last_z = p[2];
            std::copy(Pc,Pc+3,rot_center);
        }

        if( inType == InputSpecialScroll ) {
            if(button_state & KeyModifierCmd) {
                const GLprecision rx = -p2 / 1000;
                const GLprecision ry = -p1 / 1000;

                Rotation<>(T_nc,rx, ry, (GLprecision)0.0);
                OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
                LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
            }else{
                const GLprecision scrolly = p2/10;

                LieSetIdentity(T_nc);
                const GLprecision t[] = { 0,0, -scrolly*100*tf};
                LieSetTranslation<>(T_nc,t);
                if( !(button_state & MouseButtonRight) && !(rot_center[0]==0 && rot_center[1]==0 && rot_center[2]==0) )
                {
                    LieSetTranslation<>(T_nc,rot_center);
                    MatMul<3,1>(T_nc+(3*3), -scrolly * zf);
                }
                OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
                LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
            }
        }else if(inType == InputSpecialRotate) {
            const GLprecision r = p1 / 20;

            GLprecision T_2c[3*4];
            Rotation<>(T_2c, (GLprecision)0.0, (GLprecision)0.0, r);
            GLprecision mrotc[3];
            MatMul<3,1>(mrotc, rot_center, (GLprecision)-1.0);
            LieApplySO3<>(T_2c+(3*3),T_2c,mrotc);
            GLprecision T_n2[3*4];
            LieSetIdentity<>(T_n2);
            LieSetTranslation<>(T_n2,rot_center);
            LieMulSE3(T_nc, T_n2, T_2c );
            OpenGlMatrix& spec = cam_state->GetModelViewMatrix();
            LieMul4x4bySE3<>(spec.m,T_nc,spec.m);
        }

    }

    void GlHandler3D::RegisterMouseButtonHandler(GlHandler3D::mouseButton_t button, const boost::function<void (Eigen::Vector3d)>& handler, GlHandler3D::keyboardButton_t keyCondition)
    {
        mouseButtonHandler_t h;
        h.button = button;
        h.handler = handler;
        h.keyboardCondition = keyCondition;

        mouseButtonHandlers.push_back(h);
    }

    void GlHandler3D::RegisterHighlightSingle(const boost::function<void (Eigen::Vector3d, bool, bool)>& handler)
    {
        highlightSingleHandler = handler;
    }
    void GlHandler3D::RegisterHighlightDrawing(const boost::function<void (Eigen::Vector3d, bool, bool)>& handler)
    {
        highlightDrawingHandler = handler;
    }

    void GlHandler3D::RegisterKeyboardButtonHandler(GlHandler3D::keyboardButtonGroup_t group, const boost::function<void (char)>& handler, GlHandler3D::keyboardButton_t keyCondition)
    {
        keyboardButtonHandler_t h;
        h.group = group;
        h.keyboardCondition = keyCondition;
        h.handler = handler;

        keyboardButtonHandlers.push_back(h);
    }

}
