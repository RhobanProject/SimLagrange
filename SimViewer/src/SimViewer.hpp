#ifndef LEPH_SIMVIEWER_SIMVIEWER_HPP
#define LEPH_SIMVIEWER_SIMVIEWER_HPP

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "Any/src/Any.hpp"
#include "Vector/src/Vector2D.hpp"
#include "SimMecha/src/Simulation.h"

namespace Leph {
namespace SimViewer {

/**
 * SimViewer
 *
 * Graphical tool for drawing 2D
 * mechanical simulations
 */
class SimViewer
{
  public:

        /**
         * Typedef for handler function
         */
    // typedef void (*HandlerFunction)(Any::Any param);
    typedef std::function<void(Any::Any param)> HandlerFunction;

        /**
         * Configuation constant
         */
        static constexpr double MASS_RADIUS = 0.1;
        static constexpr double SEGMENT_SIZE = 0.05;
        static constexpr double CAMERA_WIDTH = 7.0;
        static constexpr double JOINT_RADIUS = 0.05;
        static constexpr double JOINT_LENGTH = 0.2;
        static constexpr double JOINT_SIZE = 0.02;
        static constexpr double TEXT_SIZE = 0.01;
        static constexpr double FRAME_SIZE = 0.02;
        static constexpr double BASE_SIZE = 0.25;
        static constexpr double BASE_THICK = 0.01;

        /**
         * Initialization with windows
         * width and height resolution
         */
        SimViewer(unsigned int width, unsigned int height) :
            _onCloseHandler(NULL),
            _onSpaceHandler(NULL),
            _onRHandler(NULL),
            _onCloseParam(),
            _onSpaceParam(),
            _onRParam(),
            _window(),
            _view(),
            _chainPos(0.0, 0.0),
            _chainAngle(0.0),
            _width(width),
            _height(height),
            _zoom(1.0),
            _button(false)
        {
            //Initiating windows
            _window.create(sf::VideoMode(width, height), "SimViewer");
            //Initialiting camera
            _ratio = (double)width/(double)height;
            _view.setCenter(sf::Vector2f(0, 0));
            _view.setSize(sf::Vector2f(CAMERA_WIDTH, CAMERA_WIDTH/_ratio));


            // _view.setSize(sf::Vector2f(CAMERA_WIDTH/_zoom, CAMERA_WIDTH/_ratio/_zoom));
            // _view.zoom(_zoom);


            _window.setView(_view);
        }

        /**
         * Event handler setter for close and space event
         * Param is given to the handle
         */
        inline void setCloseHandler(HandlerFunction handler, Any::Any param)
        {
            _onCloseHandler = handler;
            _onCloseParam = param;
        }
        inline void setSpaceHandler(HandlerFunction handler, Any::Any param)
        {
            _onSpaceHandler = handler;
            _onSpaceParam = param;
        }

        inline void setRHandler(HandlerFunction handler, Any::Any param)
        {
            _onRHandler = handler;
            _onRParam = param;
        }

        /**
         * Return true if the windows is still open
         */
        inline bool isOpen() const
        {
            return _window.isOpen();
        }

        inline void moveCam(float x, float y)
        {
            // afterCoord=_window.mapPixelToCoords(sf::Vector2i(pos.x(),pos.y()));
            afterCoord=sf::Vector2f(x,y);
            const sf::Vector2f offsetCoords(beforeCoord - afterCoord);
            sf::View view(_window.getView());
            view.move(offsetCoords);
            _window.setView(view);
            beforeCoord=afterCoord;
        }

        /**
         * Poll and handle inputs events
         */
        inline void beginDraw()
        {
            //Poll and handle events
            sf::Event event;
            while (_window.pollEvent(event)) {
                //Close event
                if (
                    event.type == sf::Event::Closed ||
                    (event.type == sf::Event::KeyPressed &&
                    event.key.code == sf::Keyboard::Escape) ||
                    (event.type == sf::Event::KeyPressed &&
                    event.key.code == sf::Keyboard::Q)
                ) {
                    _window.close();
                    if (_onCloseHandler != NULL) {
                        _onCloseHandler(_onCloseParam);
                    }
                }
                //Space event
                if (
                    event.type == sf::Event::KeyPressed &&
                    event.key.code == sf::Keyboard::Space
                ) {
                    if (_onSpaceHandler != NULL) {
                        _onSpaceHandler(_onSpaceParam);
                    }
                }

                if (
                    event.type == sf::Event::KeyPressed &&
                    event.key.code == sf::Keyboard::R
                ) {
                    if (_onRHandler != NULL) {
                        _onRHandler(_onRParam);
                    }
                }


                    //mouse
                if (event.type == sf::Event::MouseMoved)
                {
                    // std::cout<<"X: "<<event.mouseMove.x<<" Y: "<<event.mouseMove.y<<std::endl;
                    if(_button)
                    {
                        afterCoord=_window.mapPixelToCoords(sf::Vector2i(event.mouseMove.x,event.mouseMove.y));
                        const sf::Vector2f offsetCoords(beforeCoord - afterCoord);
                        sf::View view(_window.getView());
                        view.move(offsetCoords);
                        _window.setView(view);
                        beforeCoord=afterCoord;

                    }

                }

                if (event.type == sf::Event::MouseButtonPressed)
                {
                    if (event.mouseButton.button == sf::Mouse::Left)
                    {

                        beforeCoord=_window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x,event.mouseButton.y));
                        // _view.move(0.0,1.0);
                        _button=true;
                        // std::cout<<"BUTTON LEFT "<<event.mouseButton.x<<" "<<event.mouseButton.y<<std::endl;
                    }
                }
                if (event.type == sf::Event::MouseButtonReleased)
                {
                    if (event.mouseButton.button == sf::Mouse::Left)
                    {

                        // std::cout<<"BUTTON LEFT RELEASED "<<event.mouseButton.x<<" "<<event.mouseButton.y<<std::endl;
                        afterCoord=_window.mapPixelToCoords(sf::Vector2i(event.mouseButton.x,event.mouseButton.y));
                        // _view.move(0.0,1.0);
                        if(_button)
                        {
                            _button=false;
                            const sf::Vector2f offsetCoords(beforeCoord - afterCoord);
                            sf::View view(_window.getView());
                            view.move(offsetCoords);
                            _window.setView(view);
                        }
                    }
                }

                if (event.type == sf::Event::MouseWheelScrolled)
                {
                    if (event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel)
                    {
                        _zoom=1.f+event.mouseWheelScroll.delta*0.1f;

                        // std::cout<<"SCROLL: "<<event.mouseWheelScroll.delta<<" zoom: "<<_zoom<<std::endl;
                        // _view.setCenter(sf::Vector2f(0, 0));
                        sf::View view(_window.getView());
                        // view.setSize(sf::Vector2f((float)CAMERA_WIDTH/_zoom, (float)CAMERA_WIDTH/_ratio/_zoom));
                        view.zoom(_zoom);
                        _window.setView(view);
                    }
                }
            }


            //Clear screen background
            _window.clear(sf::Color::Black);

        }


    inline void plot(std::vector<Vector::Vector2D<double>> data, double minrange, double maxrange, int timewin=1000) //TODO
        {
            // Vector2D size=Vector2D(_width,_height);
                //top right
            // Vector2D centerplot=Vector2D(size.x()/2+size.x()/4,size.y()/2+size.y()/4);
            // Vector2D centerplot=Vector2D(990,215);

            int x=_width/2+_width/4;
            int y=_height/4; //Why?

            // sf::View view;

            double ratio=0.015; //bah?
            // std::cout<<"debug: "<<_width<<" "<<_height<<std::endl;
            // view.reset(sf::FloatRect(centerplot.x(), centerplot.y(), size.x()/4,size.y()/4));

            // sf::Vector2f sizef=_window.mapPixelToCoords(sf::Vector2i(_width,_height));
            sf::Vector2f centerplotf=_window.mapPixelToCoords(sf::Vector2i(x,y));

            // viewer._window.setView(view);
            drawRect(centerplotf.x,-centerplotf.y,_width/8*ratio,_height/8*ratio,0.01,sf::Color::Red);
            // viewer._window.setView(viewer._view);

            double rangeratio=(maxrange-minrange)/(_height/8*ratio);
            double timeratio=(timewin)/(_width/8*ratio);

            Vector::Vector2D<double> orig=Vector::Vector2D<double>(centerplotf.x-(_width/8*ratio)/2.0,-centerplotf.y-(_height/8*ratio)/2.0);

            if(data.size()<timewin && data.size()>0)
            {
                double prevx=data[0].x();
                // double prevx=0.0;

                double prevy=data[0].y();

                for(int i=1; i<data.size();i++)
                {

                    double oldx=(prevx-data[0].x())/timeratio-orig.x();
                    double oldy=(prevy-minrange)/rangeratio-orig.y();
                    double newx=(data[i].x()-data[0].x())/timeratio+orig.x();
                    double newy=(data[i].y()-minrange)/rangeratio-orig.y();
                    drawLineByEnds(oldx, -oldy, newx, -newy,0.1, sf::Color::Blue);
                    prevx=data[i].x();
                    prevy=data[i].y();

                    std::cout<<"debug: "<<newx<<" "<<newy<<" "<<rangeratio<<" "<<timeratio<<" "<<data[i].y()<<" "<<orig<<std::endl;
                }
            }
        }


        /**
         * Draw a reference frame at the origine or
         * at given coordinate and angle (in degree)
         */
        inline void drawFrame(double scale = 1.0,
            double x = 0.0, double y = 0.0, double angle = 0.0)
        {
            double endX = x + scale*cos(angle*M_PI/180.0);
            double endY = y + scale*sin(angle*M_PI/180.0);
            double endX2 = x + scale*cos((angle+90.0)*M_PI/180.0);
            double endY2 = y + scale*sin((angle+90.0)*M_PI/180.0);

            drawLineByEnds(x, y, endX, endY,
                FRAME_SIZE, sf::Color::Red);
            drawLineByEnds(x, y, endX2, endY2,
                FRAME_SIZE, sf::Color::Green);
            drawText("X", endX, endY, sf::Color(255, 0, 0, 100));
            drawText("Y", endX2, endY2, sf::Color(0, 255, 0, 100));
        }

        /**
         * Draw a mass at the given position or at
         * the current chain position
         */
        inline void drawMass(double x, double y, double val = -1)
        {
            drawCircle(x, y, MASS_RADIUS, sf::Color::Blue);

            //Print mass value
            if (val > 0) {
                std::ostringstream oss;
                oss << val;
                drawText(oss.str(), x, y,
                    sf::Color(0, 0, 255, 100));
            }
        }
        inline void drawMass()
        {
            drawMass(_chainPos.x, _chainPos.y);
        }

        /**
         * Draw a segment (link) from given position, length
         * and angle or from current chain position and angle
         */
        inline void drawSegment(double x, double y,
                                double length, double angle, double size=SEGMENT_SIZE, sf::Color color=sf::Color::Green)
        {
            drawLineByPolar(x, y, length, angle,
                            size, color);
        }
        inline void drawSegmentByEnd(double x1, double y1,
                                     double x2, double y2, double size=SEGMENT_SIZE, sf::Color color=sf::Color(0, 255, 0, 50))
        {
            drawLineByEnds(x1, y1, x2, y2,
                size, color);
        }

        inline void drawSegment(double length)
        {
            drawSegment(_chainPos.x, _chainPos.y, length, _chainAngle);
            _chainPos.x += length*cos(_chainAngle*M_PI/180.0);
            _chainPos.y += length*sin(_chainAngle*M_PI/180.0);
        }

        /**
         * Draw a join at given position or current chain position
         * Angle is the joint zero position (in degree)
         * Theta is the joint value (in degree)
         */
        inline void drawJoint(double x, double y, double angle, double theta)
        {
            drawCircle(x, y, JOINT_RADIUS, sf::Color::White);
            drawLineByPolar(x, y, JOINT_LENGTH, angle,
                1.5*JOINT_SIZE, sf::Color::White);
            drawLineByPolar(x, y, JOINT_LENGTH, angle+theta,
                JOINT_SIZE, sf::Color::White);

            //Print joint position
            std::ostringstream oss;
            oss.precision(3);
            oss << angleNormalize(theta);
            drawText(oss.str(), x, y, sf::Color(255, 255, 255, 150));
        }
        inline void drawJoint(double theta)
        {
            drawJoint(_chainPos.x, _chainPos.y, _chainAngle, theta);
            _chainAngle += theta;
        }

        /**
         * Draw a linear joint at given end position
         * with given value
         */
        inline void drawLinearJoint(double x1, double y1,
            double x2, double y2, double value)
        {
            drawCircle(x1, y1, JOINT_RADIUS, sf::Color::White);
            drawCircle(x2, y2, JOINT_RADIUS, sf::Color::White);

            drawLineByEnds(x1, y1, x2, y2,
                JOINT_SIZE, sf::Color::White);

            //Print joint value
            std::ostringstream oss;
            oss.precision(3);
            oss << value;
            drawText(oss.str(), x1, y1, sf::Color(255, 255, 255, 150));
        }

        inline void drawBase(double x = 0.0, double y = 0.0)
        {
            drawRect(x, y, BASE_SIZE, BASE_SIZE,
                BASE_THICK, sf::Color::White);
            drawLineByEnds(x-BASE_SIZE/2.0, y-BASE_SIZE/2.0,
                x+BASE_SIZE/2.0, y+BASE_SIZE/2.0,
                BASE_THICK, sf::Color::White);
            drawLineByEnds(x-BASE_SIZE/2.0, y+BASE_SIZE/2.0,
                x+BASE_SIZE/2.0, y-BASE_SIZE/2.0,
                BASE_THICK, sf::Color::White);

            //Print Base position
            std::ostringstream oss;
            oss.precision(3);
            oss << x << "," << y;
            drawText(oss.str(), x, y, sf::Color(255, 255, 255, 150));
        }

        /**
         * Start a new kinematic chain drawing
         * Initial position and angle can be given
         */
        inline void beginChain
            (double x = 0.0, double y = 0.0, double angle = 0.0)
        {
            _chainPos = sf::Vector2f(x, y);
            _chainAngle = angle;
        }

        /**
         * Render the screen and sleep the number
         * of given milliseconds
         */
        inline void endDraw(long millis = -1)
        {


            //Display (double buffering)
            _window.display();
            //Reset chain
            _chainPos = sf::Vector2f(0.0, 0.0);
            _chainAngle = 0.0;
            //Sleep
            if (millis != -1) {
                sf::sleep(sf::milliseconds(millis));
            }
        }

        /**
         * Draw a circle of given center, radius and color
         */
        inline void drawCircle(double x, double y,
            double radius, sf::Color color)
        {
            sf::CircleShape circle(radius);
            circle.setPosition(x-radius, -y-radius);
            circle.setFillColor(color);
            _window.draw(circle);
        }


            /**
         * Draw a line of given ends or polation position,
         * its thickness and color
         */
        inline void drawLineByEnds(double x1, double y1,
            double x2, double y2, double size, sf::Color color)
        {
            double length = sqrt(pow(x2-x1, 2)+pow(y2-y1, 2));
            double angle = atan2(y2-y1, x2-x1)*180.0/M_PI;
            drawLineByPolar(x1, y1, length, angle, size, color);
        }
        inline void drawLineByPolar(double x, double y,
            double length, double angle, double size, sf::Color color)
        {
            sf::RectangleShape rectangle(sf::Vector2f(length, size));
            rectangle.setOrigin(0, size/2);
            rectangle.setRotation(-angle);
            rectangle.move(x, -y);
            rectangle.setFillColor(color);
            _window.draw(rectangle);
        }

        /**
         * Draw a rectangle shape og given size with its center
         * at given position
         */
        inline void drawRect(double x, double y,
            double width, double height, double size, sf::Color color)
        {
            sf::RectangleShape rectangle(sf::Vector2f(width, height));
            rectangle.setOrigin(width/2.0, height/2.0);
            rectangle.move(x, -y);
            rectangle.setOutlineThickness(size);
            rectangle.setOutlineColor(color);
            rectangle.setFillColor(sf::Color::Transparent);
            _window.draw(rectangle);
        }

        /**
         * Draw the given text at given position and color
         */
        inline void drawText(const std::string& str,
            double x, double y, sf::Color color)
        {
            sf::Font font;
            if (!font.loadFromFile("../Assets/DS-DIGII.TTF")) {
                throw std::runtime_error("SimViewer unable to load font");
            }

            sf::Text text;
            text.setFont(font);
            text.setString(str);
            text.setColor(color);
            text.setPosition(x, -y);
            text.setCharacterSize(24);
            text.scale(TEXT_SIZE, TEXT_SIZE);
            _window.draw(text);
        }

        /**
         * Normalize the given angle between -180 et 180
         */
        inline double angleNormalize(double angle) const
        {
            while (angle <= -180.0) {
                angle += 360;
            }
            while (angle > 180) {
                angle -= 360;
            }
            if (fabs(angle) < 0.0001) {
                return 0.0;
            }

            return angle;
        }

//    private: //F*ck this

        /**
         * Handler function pointer for
         * close and space event
         */
        HandlerFunction _onCloseHandler;
        HandlerFunction _onSpaceHandler;
        HandlerFunction _onRHandler;

        /**
         * Parameter for close and space event
         */
        Any::Any _onCloseParam;
        Any::Any _onSpaceParam;
        Any::Any _onRParam;

        /**
         * SFML windows and camera
         */
        sf::RenderWindow _window;
        sf::View _view;

        /**
         * To handle mouse scroll zooming
         */
        double _zoom, _ratio;
        bool _button ;
        int _width, _height;
        sf::Vector2f beforeCoord, afterCoord;

        /**
         * Position and angle of chain end for segment drawing
         */
        sf::Vector2f _chainPos;
        double _chainAngle;


};

}
}

#endif
