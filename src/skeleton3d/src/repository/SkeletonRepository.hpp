#ifndef SKELETONREPOSITORY_HPP
#define SKELETONREPOSITORY_HPP

#include <skeleton3d/Skeleton3d.h>
#include "RepositoryDataStructures.hpp"
#include "SkeletonMatcher.hpp"

/**
 * Repository of Skeleton Information collected from all sensors.
 *
 * Sensors (with skeleton generators) should send their information so that this repository can
 * collect them and establish and maintain the masterlist of all currently recognised skeletons.
 * This skeleton should, for all external users, represent the "single source of truth" - meaning
 * that external users should retrieve the skeleton information from this repository instead of
 * from individual sensors.
 *
 * When new information about one or more skeletons is received the repository decides on how to
 * use this information. If a skeleton is in close proximity to one already present in the
 * masterlist chances are that it's the same skeleton. Only distance features into this decision,
 * not current pose. The distance at which two skeletons are considered to be the same can be
 * set by the decay_strength parameter of the constructor.
 *
 * Information also ages. The longer ago the last information about a skeleton body part has been
 * received, the less confident the repository is that this is still accurate information. Once
 * confidence has dropped to zero the respective body part will be removed from the masterlist
 * until, eventually, the whole skeleton disappears. The strength at which the information ages
 * can be set by changing the decay_strength parameter in the constructor.
 */
class SkeletonRepository {
    using Skeleton = skeleton3d::Skeleton3d;
    template<class T> using vector = std::vector<T>;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    
public:
    /**
     * Constructor.
     *
     * @param decay_strength Higher strength shortens time before skeleton parts start to
     *                       disapper if no new message about them has been received. Setting
     *                       this to 0.0 disables aging.
     */
    SkeletonRepository(double distance_threshold, double decay_strength) :
        DECAY_STRENGTH_(decay_strength),
        skeleton_id_(0),
        skeleton_matcher_(distance_threshold)
    {}
    ~SkeletonRepository() {};
    
    /**
     * Update skeleton masterlist with new skeleton information.
     *
     * New skeletons get added into the repository and existing ones will be updated (decision
     * will be based on distance).
     *
     * @param skeletons_msg Message with new skeleton information.
     */
    void update_skeletons(std::vector<TimedSkeleton> &observed_skeletons);

    /**
     * Retrieve all current information from this repository.
     *
     * \note Every call of this method will also initiate decay of existing information. Decay
     *       is based on time, so the frequency of calling this method does not affect aging.
     *
     * @return The current skeleton masterlist.
     */
    std::vector<Skeleton> get_skeleton_masterlist();


private:
    class NoCenterpointFoundError : public std::exception { };

    /** Strength multiplier with which information decay happens */
    const double DECAY_STRENGTH_;
    /** Masterlist holding all current skeletons */
    std::vector<TimedSkeleton> skeletons_masterlist_;
    unsigned int skeleton_id_;
    SkeletonMatcher skeleton_matcher_;
    
    void add_to_masterlist(const vector<TimedSkeleton*>& new_tracks);
    
    /*
     * Insert a new skeleton that has not yet been introduced into the masterlist.
     *
     * @param new_skeleton The skeleton to insert a copy of into the masterlist.
     * @param timestamp The timestamp of the message the new_skeleton has been received in.
     */
    void add_to_masterlist(const TimedSkeleton &new_skeleton);
    
    /**
     * Transform a TimedSkeleton into a normal Skeleton by ignoring body part timestamps.
     */
    Skeleton simple_skeleton_from(const TimedSkeleton &timed_skeleton);

    /*
     * Decay the masterlist by aging the information.
     *
     * Aging is accomplished by reducing the confidence of the individual body parts depending
     * on how much time has passed since that body part information has been received and on
     * DECAY_STRENGTH_. Once all body parts have decayed completely the skeleton is removed from
     * the masterlist.
     */
    void decay_masterlist();

    /**
     * Decay one skeleton. See decay_masterlist() for more information.
     *
     * @param skeleton_information Reference to the skeleton in the masterlist to age.
     * @return true iff the skeleton has no valid body parts anymore and needs to be removed.
     */
    bool decay_skeleton(TimedSkeleton &timed_skeleton);

    /**
     * Decay one bodypart. See decay_masterlist() for more information. Sets the body part
     * to invalid if the confidence has reached zero.
     *
     * @param body_part_info Reference to the body part of a skeleton in the masterlist to age.
     * @return true iff the body part has become invalid because confidence reached 0.
     */
    bool decay_bodypart(TimedBodyPart &timed_body_part);
};

#endif
