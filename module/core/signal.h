#ifndef SCAN3D_CORE_SIGNAL_H
#define SCAN3D_CORE_SIGNAL_H


#include <unordered_map>
#include <functional>


namespace scan3d {

    /**
     * \brief Implementation of a simple signal-slot mechanism.
     *        Multiple slots (a class and it member function) can be connected
     *        to a signal object.
     *        You can connect functions to the signal which will be called when
     *        the trigger() method on the signal object is invoked. Any argument
     *        passed to emit() will be passed to the given functions.
     *        A typical usage of Signal in Scan3D is camera manipulation. When
     *        the camera has been manipulated, the viewer should be notified (e.g.,
     *        a repaint event should be triggered). This is done by calling
     *        to the viewer's update() function. So in Scan3D, the viewer's update
     *        function is connected to the camera.
     * \attention Current implementation can hold only one single function of each owner.
     *            (can be easily extended to multiple ones if needed).
     * \todo Extend this class to accept non-member functions and multiple functions of a class.
     */
    class Signal {
	public:
        /**
         * \brief Connect a slot to this signal.
         * \example
         *        - If no overloaded function:
         *              camera()->connect(this, &Viewer::update);
         *        - If the function has been overloaded (of course you can always treat it this way):
         *              camera()->connect(this, static_cast<void(Viewer::*)(void) const>(&Viewer::update));
         */
        template < class Class, class Function, class... Args >
        void connect(Class&& owner, Function&& func, Args&&... args)  {
            if (owner && func)
                slots_[owner] = std::bind(func, owner, std::forward<Args>(args)...);
        }

        /**
         * \brief Disconnect a slot from this signal.
         */
        template < class Class >
        void disconnect(Class&& owner)  {
            slots_.erase(owner);
        }

        /**
         * Trigger all the connected slots.
         * \todo A better function name can be 'emit', but occupied by other software like Qt.
         *       Don't know how to still use 'emit'.
         */
        template < class ... Args >
        void trigger(Args&&... args) {
            for(auto it : slots_) {
                it.second(std::forward<Args>(args)...);
            }
        }

    private:
        std::unordered_map<void*, std::function<void(void)>> slots_;
	};

}

#endif // SCAN3D_CORE_SIGNAL_H
