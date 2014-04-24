/**
 * @file   state_estimator.h
 * @author George Andrew Brindeiro (georgebrindeiro@lara.unb.br)
 * @date   Apr 24, 2014
 *
 * @attention Copyright (C) 2014
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

#ifndef LARA_RGBD_STATE_ESTIMATOR_H_
#define LARA_RGBD_STATE_ESTIMATOR_H_

class StateEstimator
{
    public:
        StateEstimator()
        {

        }

        ~SensorProcessor() {}

        /*!
        * @brief
        *
        * This does
        *
        * @param bar                    ef
        */
        void foo(int bar);

    private:
        int var_;      /**< Indicates which feature detector we are using to process point clouds  */
};

#endif  // LARA_RGBD_STATE_ESTIMATOR_H_
