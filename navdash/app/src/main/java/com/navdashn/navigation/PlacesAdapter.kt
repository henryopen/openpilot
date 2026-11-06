package com.navdashn.navigation

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.recyclerview.widget.DiffUtil
import androidx.recyclerview.widget.ListAdapter
import androidx.recyclerview.widget.RecyclerView
import com.navdashn.databinding.ItemPlaceBinding

/**
 * 地點列表 Adapter
 * @param onPlaceClick 點擊地點時的回調 (開始導航)
 * @param onPlaceLongClick 長按地點時的回調 (加入/移除常用)，返回 true 表示已處理
 */
class PlacesAdapter(
    private val onPlaceClick: (PlaceItem) -> Unit,
    private val onPlaceLongClick: ((PlaceItem) -> Boolean)? = null
) : ListAdapter<PlaceItem, PlacesAdapter.PlaceViewHolder>(PlaceDiffCallback()) {

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): PlaceViewHolder {
        val binding = ItemPlaceBinding.inflate(
            LayoutInflater.from(parent.context),
            parent,
            false
        )
        return PlaceViewHolder(binding)
    }

    override fun onBindViewHolder(holder: PlaceViewHolder, position: Int) {
        holder.bind(getItem(position))
    }

    inner class PlaceViewHolder(
        private val binding: ItemPlaceBinding
    ) : RecyclerView.ViewHolder(binding.root) {

        fun bind(place: PlaceItem) {
            binding.placeIcon.text = place.icon
            binding.placeName.text = place.name
            binding.placeAddress.text = place.address

            binding.root.setOnClickListener {
                onPlaceClick(place)
            }

            binding.root.setOnLongClickListener {
                onPlaceLongClick?.invoke(place) ?: false
            }

            binding.navButton.setOnClickListener {
                onPlaceClick(place)
            }
        }
    }

    private class PlaceDiffCallback : DiffUtil.ItemCallback<PlaceItem>() {
        override fun areItemsTheSame(oldItem: PlaceItem, newItem: PlaceItem): Boolean {
            return oldItem.id == newItem.id
        }

        override fun areContentsTheSame(oldItem: PlaceItem, newItem: PlaceItem): Boolean {
            return oldItem == newItem
        }
    }
}
