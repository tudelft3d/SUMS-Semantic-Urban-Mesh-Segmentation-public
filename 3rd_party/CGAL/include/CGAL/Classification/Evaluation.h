// Copyright (c) 2017 GeometryFactory Sarl (France).
// All rights reserved.
//
// This file is part of CGAL (www.cgal.org).
// You can redistribute it and/or modify it under the terms of the GNU
// General Public License as published by the Free Software Foundation,
// either version 3 of the License, or (at your option) any later version.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
// $URL: https://github.com/CGAL/cgal/blob/releases/CGAL-4.14/Classification/include/CGAL/Classification/Evaluation.h $
// $Id: Evaluation.h 82af495 %aI Simon Giraudot
// SPDX-License-Identifier: GPL-3.0+
//
// Author(s)     : Simon Giraudot

#ifndef CGAL_CLASSIFICATION_EVALUATION_H
#define CGAL_CLASSIFICATION_EVALUATION_H

#include <CGAL/license/Classification.h>

#include <CGAL/Classification/Label.h>
#include <CGAL/Classification/Label_set.h>
#include <map>
#include <cmath> // for std::isnan

namespace CGAL {

namespace Classification {

/*!
  \ingroup PkgClassificationDataStructures

  \brief Class to compute several measurements to evaluate the quality
  of a classification output.
*/
inline float value_validation_check(const float value)
{
	if (!std::isnormal(value) || value < 10E-4  || value > 1.0f)
		return 0.0f;
	else
		return value;
}

class Evaluation
{
  mutable std::map<Label_handle, std::size_t> m_map_labels;

  std::vector<float> m_precision;
  std::vector<float> m_recall;
  std::vector<float> m_iou; // intersection over union
  float m_mean_accuracy;
  float m_accuracy;
  float m_mean_iou;
  float m_mean_f1;

public:

  /// \name Constructor
  /// @{

  
/*!

  \brief Instantiates an evaluation object and computes all
  measurements.

  \param labels labels used.

  \param ground_truth vector of label indices: it should contain the
  index of the corresponding label in the `Label_set` provided in the
  constructor. Input items that do not have a ground truth information
  should be given the value `-1`.

  \param result similar to `ground_truth` but contained the result of
  a classification.

*/
  template <typename GroundTruthIndexRange, typename ResultIndexRange>
  Evaluation (const Label_set& labels,
              const GroundTruthIndexRange& ground_truth,
              const ResultIndexRange& result)
    : m_precision (labels.size()),
      m_recall (labels.size()),
      m_iou (labels.size())
  {
    for (std::size_t i = 0; i < labels.size(); ++ i)
      m_map_labels[labels[i]] = i;

    std::vector<std::size_t> true_positives (labels.size());
    std::vector<std::size_t> false_positives (labels.size());
    std::vector<std::size_t> false_negatives (labels.size());

    std::size_t sum_true_positives = 0;
    std::size_t total = 0;
    
    for (std::size_t j = 0; j < ground_truth.size(); ++ j)
    {
      int gt = static_cast<int>(ground_truth[j]);
      int res = static_cast<int>(result[j]);
      if (gt == -1 || res == -1)
        continue;
      ++ total;
      if (gt == res)
      {
        ++ true_positives[gt];
        ++ sum_true_positives;
        continue;
      }
      ++ false_positives[res];
      ++ false_negatives[gt];
    }

    m_mean_iou = 0.;
    m_mean_f1 = 0.;

    std::size_t correct_labels = 0;
    
    for (std::size_t j = 0; j < labels.size(); ++ j)
    {
      m_precision[j] = true_positives[j] / float(true_positives[j] + false_positives[j]);
      m_recall[j] = true_positives[j] / float(true_positives[j] + false_negatives[j]);
      m_iou[j] = true_positives[j] / float(true_positives[j] + false_positives[j] + false_negatives[j]);

      if (std::isnan(m_iou[j]))
        continue;

      ++ correct_labels;
      m_mean_iou += m_iou[j];
      m_mean_f1 += 2.f * (m_precision[j] * m_recall[j])
        / (m_precision[j] + m_recall[j]);
    }

    m_mean_iou /= correct_labels;
    m_mean_f1 /= correct_labels;
    m_accuracy = sum_true_positives / float(total);
  }

  /// @}

  /// \name Label Evaluation
  /// @{
	  
    //**********************************Weixiao GAO update*****************************************
  template <typename GroundTruthIndexRange, typename ResultIndexRange>
  Evaluation(const Label_set& labels,
      const GroundTruthIndexRange& ground_truth,
      const ResultIndexRange& result,
      const std::vector<float> weight)
      : m_precision(labels.size()),
      m_recall(labels.size()),
      m_iou(labels.size())
  {
      for (std::size_t i = 0; i < labels.size(); ++i)
          m_map_labels[labels[i]] = i;

      std::vector<double> true_positives(labels.size());
      std::vector<double> false_positives(labels.size());
      std::vector<double> false_negatives(labels.size());

      double sum_true_positives = 0;
      double total = 0;

      for (std::size_t j = 0; j < ground_truth.size(); ++j)
      {
          int gt = static_cast<int>(ground_truth[j]);
          int res = static_cast<int>(result[j]);
          if (gt == -1 || res == -1)
              continue;
          total += weight[j];
          if (gt == res)
          {
              true_positives[gt] += weight[j];
              sum_true_positives += weight[j];
              continue;
          }

          false_positives[res] += weight[j];
          false_negatives[gt] += weight[j];
      }

      m_mean_iou = 0.;
      m_mean_f1 = 0.;

      std::size_t correct_labels = 0;

      for (std::size_t j = 0; j < labels.size(); ++j)
      {
          m_precision[j] = true_positives[j] / float(true_positives[j] + false_positives[j]);
          m_recall[j] = true_positives[j] / float(true_positives[j] + false_negatives[j]);
          m_iou[j] = true_positives[j] / float(true_positives[j] + false_positives[j] + false_negatives[j]);

          if (std::isnan(m_iou[j])
			  || std::isinf(m_iou[j]))
              continue;

          ++correct_labels;
		  m_mean_accuracy += m_recall[j];
          m_mean_iou += m_iou[j];
          m_mean_f1 += 2.f * (m_precision[j] * m_recall[j])
              / (m_precision[j] + m_recall[j]);
      }

      m_mean_iou /= correct_labels;
      m_mean_f1 /= correct_labels;
      m_accuracy = sum_true_positives / total;
	  m_mean_accuracy /= float(labels.size());
  }
  //*********************************************************************************************


  /*!

    \brief Returns the precision of the training for the given label.

    Precision is the number of true positives divided by the sum of
    the true positives and the false positives.

  */
  float precision (Label_handle label) const
  {
    return value_validation_check(m_precision[m_map_labels[label]]);
  }

  /*!

    \brief Returns the recall of the training for the given label.

    Recall is the number of true positives divided by the sum of
    the true positives and the false negatives.

  */
  float recall (Label_handle label) const
  {
    return value_validation_check(m_recall[m_map_labels[label]]);
  }

  /*!

    \brief Returns the \f$F_1\f$ score of the training for the given label.

    \f$F_1\f$ score is the harmonic mean of `precision()` and `recall()`:

    \f[
    F_1 = 2 \times \frac{precision \times recall}{precision + recall}
    \f]

  */
  float f1_score (Label_handle label) const
  {
    std::size_t label_idx = m_map_labels[label];
    return value_validation_check(2.f * (m_precision[label_idx] * m_recall[label_idx])
      / (m_precision[label_idx] + m_recall[label_idx]));
  }

  /*!
    \brief Returns the intersection over union of the training for the
    given label.

    Intersection over union is the number of true positives divided by
    the sum of the true positives, of the false positives and of the
    false negatives.
  */
  float intersection_over_union (Label_handle label) const
  {
    return value_validation_check(m_iou[m_map_labels[label]]);
  }

  /// @}
  
  /// \name Global Evaluation
  /// @{

  
  /*!
    \brief Returns the accuracy of the training.

    Accuracy is the total number of true positives divided by the
    total number of provided inliers.
  */
  float accuracy() const { return value_validation_check(m_accuracy); }
  
  /*!
    \brief Returns the mean \f$F_1\f$ score of the training over all
    labels (see `f1_score()`).
  */
  float mean_f1_score() const { return value_validation_check(m_mean_f1); }
  
  /*!
    \brief Returns the mean intersection over union of the training
    over all labels (see `intersection_over_union()`).
  */
  float mean_intersection_over_union() const { return value_validation_check(m_mean_iou); }


  /*!

  \brief Returns the mAcc of the training for the given label.

  mAcc is the sum of recall divided by the total number of
  labels.

*/
  float mean_accuracy() const  {  return value_validation_check(m_mean_accuracy); }

  /// @}
  
};
  
  
} // namespace Classification

} // namespace CGAL

#endif // CGAL_CLASSIFICATION_EVALUATION_H

