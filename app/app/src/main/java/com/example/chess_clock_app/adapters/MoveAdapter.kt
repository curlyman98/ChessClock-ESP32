package com.example.chess_clock_app.adapters

import android.annotation.SuppressLint
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.TextView
import androidx.recyclerview.widget.RecyclerView
import com.example.chess_clock_app.R
import com.example.chess_clock_app.viewmodels.MoveCell
import com.example.chess_clock_app.viewmodels.MoveRow

class MoveAdapter : RecyclerView.Adapter<MoveAdapter.ViewHolder>() {

    private var rows: List<MoveRow> = emptyList()

    // Unlike GameAdapter, this one is created once and gets new rows after every move,
    // so the list keeps its scroll position instead of jumping back to the top
    @SuppressLint("NotifyDataSetChanged")
    fun updateRows(newRows: List<MoveRow>) {
        rows = newRows
        notifyDataSetChanged()
    }

    //How a single row of the list looks like
    class ViewHolder(view: View) : RecyclerView.ViewHolder(view) {

        val moveNumber: TextView = view.findViewById(R.id.moveNumber)

        val whiteMove: View = view.findViewById(R.id.whiteMove)
        val whiteSan: TextView = view.findViewById(R.id.whiteSan)
        val whiteClock: TextView = view.findViewById(R.id.whiteClock)

        val blackMove: View = view.findViewById(R.id.blackMove)
        val blackSan: TextView = view.findViewById(R.id.blackSan)
        val blackClock: TextView = view.findViewById(R.id.blackClock)
    }

    override fun onCreateViewHolder(
        parent: ViewGroup,
        viewType: Int
    ): ViewHolder {
        val view = LayoutInflater
            .from(parent.context)
            .inflate(R.layout.item_move, parent, false)

        return ViewHolder(view)
    }

    override fun onBindViewHolder(
        holder: ViewHolder,
        position: Int
    ) {
        val row = rows[position]

        holder.moveNumber.text = "${row.number}."
        bindCell(row.white, holder.whiteMove, holder.whiteSan, holder.whiteClock)
        bindCell(row.black, holder.blackMove, holder.blackSan, holder.blackClock)
    }

    override fun getItemCount(): Int {
        return rows.size
    }

    private fun bindCell(cell: MoveCell?, container: View, san: TextView, clock: TextView) {
        // INVISIBLE instead of GONE: the empty cell keeps its width, so the columns stay aligned
        if (cell == null) {
            container.visibility = View.INVISIBLE
            return
        }
        container.visibility = View.VISIBLE

        // "…" = there is a clock time for this move, but the move itself isn't entered yet
        san.text = cell.san ?: "…"
        clock.text = cell.clock?.let(::formatClock) ?: ""
        container.setBackgroundResource(if (cell.isCurrent) R.drawable.current_move_background else 0)
    }

    // "0:04:58" -> "4:58", "1:02:03" stays "1:02:03"
    private fun formatClock(clock: String): String {
        val parts = clock.split(":")
        if (parts.size != 3 || parts[0].toIntOrNull() != 0) return clock
        return "${parts[1].toInt()}:${parts[2]}"
    }
}
