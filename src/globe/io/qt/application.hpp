#ifndef GLOBEART_SRC_GLOBE_IO_QT_APPLICATION_HPP_
#define GLOBEART_SRC_GLOBE_IO_QT_APPLICATION_HPP_

#include <CGAL/Basic_viewer.h>
#include <QtWidgets/QApplication>
#include <memory>

namespace globe::io::qt {

class Application {
public:
    Application(int argc, char *argv[]);
    int run();

private:
    std::unique_ptr<QApplication> _app;
};

inline Application::Application(int argc, char *argv[]) {
    ::CGAL::Qt::init_ogl_context(4, 3);
    _app = std::make_unique<QApplication>(argc, argv);
}

inline int Application::run() {
    return _app->exec();
}

} // namespace globe::io::qt

#endif //GLOBEART_SRC_GLOBE_IO_QT_APPLICATION_HPP_
