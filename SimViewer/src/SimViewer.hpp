#ifndef LEPH_SIMVIEWER_SIMVIEWER_HPP
#define LEPH_SIMVIEWER_SIMVIEWER_HPP

#include <cmath>
#include <sstream>
#include <stdexcept>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "Any/src/Any.hpp"

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
        typedef void (*HandlerFunction)(Any::Any param);

        /**
         * Configuation constant
         */
        static const double PI = 3.141592653589793;
        static const double MASS_RADIUS = 0.1;
        static const double SEGMENT_SIZE = 0.05;
        static const double CAMERA_WIDTH = 7.0;
        static const double JOINT_RADIUS = 0.05;
        static const double JOINT_LENGTH = 0.2;
        static const double JOINT_SIZE = 0.02;
        static const double TEXT_SIZE = 0.01;
        static const double FRAME_SIZE = 0.02;

        /**
         * Initialization with windows 
         * width and height resolution
         */
        SimViewer(unsigned int width, unsigned int height) :
            _onCloseHandler(NULL),
            _onSpaceHandler(NULL),
            _onCloseParam(),
            _onSpaceParam(),
            _window(),
            _view(),
            _chainPos(0.0, 0.0),
            _chainAngle(0.0)
        {
            //Initiating windows
            _window.create(sf::VideoMode(width, height), "SimViewer");
            //Initialiting camera
            double ratio = (double)width/(double)height;
            _view.setCenter(sf::Vector2f(0, 0));
            _view.setSize(sf::Vector2f(CAMERA_WIDTH, CAMERA_WIDTH/ratio));
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

        /**
         * Return true if the windows is still open
         */
        inline bool isOpen() const
        {
            return _window.isOpen();
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
            }
            //Clear screen background
            _window.clear(sf::Color::Black);
        }

        /**
         * Draw a reference frame at the origine or
         * at given coordinate
         */
        inline void drawFrame(double scale = 1.0, double x = 0.0, double y = 0.0)
        {
            drawLineByEnds(x, y, x+scale, y, 
                FRAME_SIZE, sf::Color::Red);
            drawLineByEnds(x, y, x, y+scale, 
                FRAME_SIZE, sf::Color::Blue);
            drawText("X", 1.0, 0.0, sf::Color::Red);
            drawText("Y", 0.0, 1.0, sf::Color::Blue);
        }

        /**
         * Draw a mass at the given position or at
         * the current chain position
         */
        inline void drawMass(double x, double y)
        {
            drawCircle(x, y, MASS_RADIUS, sf::Color::Blue);
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
            double length, double angle)
        {
            drawLineByPolar(x, y, length, angle, 
                SEGMENT_SIZE, sf::Color::Green);
        }
        inline void drawSegment(double length)
        {
            drawSegment(_chainPos.x, _chainPos.y, length, _chainAngle);
            _chainPos.x += length*cos(_chainAngle*PI/180.0);
            _chainPos.y += length*sin(_chainAngle*PI/180.0);
        }

        /**
         * Draw a join at given position or current chain position
         * Angle is the joint zero position (in degree)
         * Theta is the joint value (in degree)
         */
        inline void drawJoint(double x, double y, double angle, double theta)
        {
            drawCircle(x, y, JOINT_RADIUS, sf::Color::Red);
            drawLineByPolar(x, y, JOINT_LENGTH, angle, 
                JOINT_SIZE, sf::Color::Red);
            drawLineByPolar(x, y, JOINT_LENGTH, angle+theta, 
                JOINT_SIZE, sf::Color::Red);
           
            //Print joint position
            std::ostringstream oss;
            oss.precision(3);
            oss << angleNormalize(theta);
            drawText(oss.str(), x, y, sf::Color::Red);
        }
        inline void drawJoint(double theta)
        {
            drawJoint(_chainPos.x, _chainPos.y, _chainAngle, theta);
            _chainAngle += theta;
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

    private:

        /**
         * Handler function pointer for
         * close and space event
         */
        HandlerFunction _onCloseHandler;
        HandlerFunction _onSpaceHandler;

        /**
         * Parameter for close and space event
         */
        Any::Any _onCloseParam;
        Any::Any _onSpaceParam;

        /**
         * SFML windows and camera
         */
        sf::RenderWindow _window;
        sf::View _view;

        /**
         * Position and angle of chain end for segment drawing
         */
        sf::Vector2f _chainPos;
        double _chainAngle;

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
            double angle = atan2(y2-y1, x2-x1)*180.0/PI;
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
         * Draw the given text at given position and color
         */
        inline void drawText(const std::string& str, 
            double x, double y, sf::Color color)
        {
            sf::Font font;
            if (!font.loadFromFile("DS-DIGII.TTF")) {
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
};

}
}

#endif

