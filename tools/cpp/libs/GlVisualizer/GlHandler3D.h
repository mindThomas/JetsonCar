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

#pragma once

#include <pangolin/display/opengl_render_state.h>
#include <pangolin/handler/handler_enums.h>
#include <pangolin/handler/handler.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>

#if defined(HAVE_EIGEN) && !defined(__CUDACC__) //prevent including Eigen in cuda files
#define USE_EIGEN
#endif

#ifdef USE_EIGEN
#include <Eigen/Core>
#endif

namespace pangolin
{

    struct PANGOLIN_EXPORT GlHandler3D : Handler
    {
        GlHandler3D(OpenGlRenderState& cam_state, AxisDirection enforce_up=AxisNone, float trans_scale=0.01f, float zoom_fraction= PANGO_DFLT_HANDLER3D_ZF);

        virtual bool ValidWinDepth(GLprecision depth);
        virtual void PixelUnproject( View& view, GLprecision winx, GLprecision winy, GLprecision winz, GLprecision Pc[3]);
        virtual void GetPosNormal(View& view, int x, int y, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision nw[3], GLprecision default_z = 1.0);
        void GetPos(pangolin::View& view, int winx, int winy, GLprecision p[3], GLprecision Pw[3], GLprecision Pc[3], GLprecision default_z);

        void Keyboard(View&, unsigned char key, int x, int y, bool pressed);
        void Mouse(View&, MouseButton button, int x, int y, bool pressed, int button_state);
        void MouseMotion(View&, int x, int y, int button_state);
        void Special(View&, InputSpecial inType, float x, float y, float p1, float p2, float p3, float p4, int button_state);
        void PassiveMouseMotion(View& display, int x, int y, int button_state);


        /* Button handlers */
        typedef enum mouseButton_t : int {
            Left = pangolin::MouseButtonLeft,
            Middle = pangolin::MouseButtonMiddle,
            Right = pangolin::MouseButtonRight,
            WheelUp = pangolin::MouseWheelUp,
            WheelDown = pangolin::MouseWheelDown,
            WheelRight = pangolin::MouseWheelRight,
            WheelLeft = pangolin::MouseWheelLeft
        } mouseButton_t;

        typedef enum keyboardButton_t : int {
            None = 0,
            Shift = pangolin::KeyModifierShift,
            Ctrl = pangolin::KeyModifierCtrl,
            Alt = pangolin::KeyModifierAlt,
            Cmd = pangolin::KeyModifierCmd,
            Fnc = pangolin::KeyModifierFnc
        } keyboardButton_t;

        typedef enum keyboardButtonGroup_t : int {
            Characters = 0,
            Numbers
        } keyboardButtonGroup_t;

        void RegisterMouseButtonHandler(mouseButton_t button, const boost::function<void (Eigen::Vector3d)>& handler, keyboardButton_t keyCondition = None);

        void RegisterHighlightSingle(const boost::function<void (Eigen::Vector3d, bool, bool)>& handler);
        void RegisterHighlightDrawing(const boost::function<void (Eigen::Vector3d, bool, bool)>& handler);

        void RegisterKeyboardButtonHandler(keyboardButtonGroup_t group, const boost::function<void (char)>& handler, keyboardButton_t keyCondition = None);

#ifdef USE_EIGEN
        // Return selected point in world coordinates
        inline Eigen::Vector3d getMouse3Dposition() const {
            return Eigen::Map<const Eigen::Matrix<GLprecision,3,1>>(Pw2).cast<double>();
        }
#endif
        inline int KeyState() const{
            return funcKeyState;
        }

    protected:
        OpenGlRenderState* cam_state;
        const static int hwin = 8;
        AxisDirection enforce_up;
        float tf; // translation factor
        float zf; // zoom fraction
        CameraSpec cameraspec;
        GLprecision last_z;
        float last_pos[2];
        GLprecision rot_center[3];

        GLprecision p[3];
        GLprecision Pw[3];
        GLprecision Pw2[3];
        GLprecision Pc[3];
        GLprecision n[3];

        int funcKeyState;

        bool prev_leftMouseBtnState;

        typedef struct mouseButtonHandler_t {
            mouseButton_t button;
            keyboardButton_t keyboardCondition;
            boost::function<void (Eigen::Vector3d)> handler;
        } mouseButtonHandler_t;

        std::vector<mouseButtonHandler_t> mouseButtonHandlers;
        boost::function<void (Eigen::Vector3d, bool, bool)> highlightSingleHandler;
        boost::function<void (Eigen::Vector3d, bool, bool)> highlightDrawingHandler;

        typedef struct keyboardButtonHandler_t {
            keyboardButtonGroup_t group;
            keyboardButton_t keyboardCondition;
            boost::function<void (char)> handler;
        } keyboardButtonHandler_t;

        std::vector<keyboardButtonHandler_t> keyboardButtonHandlers;
    };

}
