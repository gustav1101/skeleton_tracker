#ifndef SKELETONREPOSITORY_HPP
#define SKELETONREPOSITORY_HPP

#include <geometry_msgs/Point.h>
#include <skeleton3d/BodyPart3d.h>
#include <skeleton3d/Skeleton3d.h>
#include <skeleton3d/Skeletons3d.h>
#include <boost/optional.hpp>
#include <ros/ros.h> // For time
#include "FrameTransformer.hpp"
#include "RepositoryDataStructures.hpp"

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
    using BodyPart = skeleton3d::BodyPart3d;
    using Skeleton = skeleton3d::Skeleton3d;
    using Point = geometry_msgs::Point;
    template<class T> using optional = boost::optional<T>;
    using TimedBodyPart = repository_data_structures::TimedBodyPart;
    using TimedSkeleton = repository_data_structures::TimedSkeleton;
    
public:
    /**
     * Constructor.
     *
     * @param position_tolerance Minimum distance in m, but using taxicap metric, between to
     *                           recognised skeletons before they are merged into one.
     * @param decay_strength Higher strength shortens time before skeleton parts start to
     *                       disapper if no new message about them has been received. Setting
     *                       this to 0.0 disables aging.
     */
    SkeletonRepository(double position_tolerance, double decay_strength) :
        POSITION_TOLERANCE_(position_tolerance),
        DECAY_STRENGTH_(decay_strength),
        skeleton_id_(0) {}
    ~SkeletonRepository() {};
    
    /**
     * Update skeleton masterlist with new skeleton information.
     *
     * New skeletons get added into the repository and existing ones will be updated (decision
     * will be based on distance).
     *
     * @param skeletons_msg Message with new skeleton information.
     */
    void update_skeletons(const std::vector<TimedSkeleton> &timed_skeletons);

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

    /** Tolerance until two skeletons are considered identical. */
    const double POSITION_TOLERANCE_;
    /** Strength multiplier with which information decay happens */
    const double DECAY_STRENGTH_;
    /** Masterlist holding all current skeletons */
    std::vector<TimedSkeleton> skeletons_masterlist_;
    std::vector<TimedSkeleton*> exclude_list_;
    unsigned int skeleton_id_;
       
    /*
     * Try to find given skeleton in the masterlist.
     *
     * Searches the repository for a skeleton that is close enough to new_skeleton to be
     * considered the same.
     *
     * @param new_skeleton A reference to the existing skeleton if one exists, boost::none else.
     */
    optional<TimedSkeleton&> find_skeleton_in_list(const TimedSkeleton &new_skeleton);

    /**
     * Decide whether or not a skeleton is identical to another.
     *
     * This is decided based on their distance to each other. Distance is measured in meters,
     * but calculated using the taxicap metric instead of euclidian distance. This improves
     * speed and since the value this is compared to is best selected by try and error anyway,
     * the euclidian distance would not help here.
     *
     * @param skel1 One skeleton
     * @param skel2 Another skeleton
     * @return true iff both skeletons are considered identical
     */
    bool is_same_skeleton(const TimedSkeleton &skel1, const TimedSkeleton &skel2);

    bool from_same_message(const TimedSkeleton &skeleton);
    /**
     * Calculate center of skeleton.
     *
     * Current calculation only uses Center head point and center torso point (part ids 0 and 1)
     * to avoid the center point rapidly changing when a person starts leaving the camera angle.
     * At least one of those two positions is recognised by the model anyway.
     *
     * @param body_parts List of all parts that may be used in calculating center position.
     * @return Center position of corresponding skeleton.
     */
    Point get_skeleton_center_position(const std::vector<TimedBodyPart> &body_parts);

    /**
     * Get mean position of given body parts.
     *
     * @param body_parts All body parts to calculate average position of.
     * @return The mean position of all body_parts.
     */
    Point calculate_mean_position(const std::vector<const TimedBodyPart*> body_parts);

    /**
     * Calculate taxicap distance between the two given points. See
     * https://en.wikipedia.org/wiki/Taxicab_geometry for details.
     */
    double distance_between_points(const Point &point1, const Point &point2);

    /**
     * Merge the new skeleton into the one already in the masterlist.
     *
     * Whether or not to update the existing skeleton is decided individually per body part.
     * This is to make sure that if one camera has a good angle on a certain body part (and thus
     * hopefully high confidence in it's position) but not on some others then the low confidence
     * body parts will be overwritten by ones with higher confidence.
     *
     * @param new_skeleton The new skeleton that is considered an update to one already in the
     *                     masterlist.
     * @param existing_skeleton The existing skeleton that the new skeleton should be merged
     *                          into.
     * @param timestamp Timestamp of the message the new_skeleton has been received in.
     *
     */
    void merge_skeleton(const TimedSkeleton &new_skeleton,
                        TimedSkeleton &exisiting_skeleton);

    /**
     * Decide whether or not to update a body part.
     *
     * Invalid body parts will always be overwritten with valid ones. Invalid parts never
     * overwrite anything. Other than that, only confidence influences the decision.
     */
    bool should_update(const TimedBodyPart &new_timed_body_part,
                       const TimedBodyPart &existing_timed_body_part);

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
