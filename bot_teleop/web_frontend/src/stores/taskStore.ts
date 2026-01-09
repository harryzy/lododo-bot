/**
 * 任务状态存储
 * 管理任务列表和状态
 */

import { create } from 'zustand'

interface Task {
  task_id: string
  type: string
  status: string
  progress: number
  created_at: string
}

interface TaskState {
  tasks: Task[]
  currentTask: Task | null
  addTask: (task: Task) => void
  updateTask: (task_id: string, updates: Partial<Task>) => void
  removeTask: (task_id: string) => void
  setCurrentTask: (task: Task | null) => void
  clearTasks: () => void
}

export const useTaskStore = create<TaskState>((set) => ({
  tasks: [],
  currentTask: null,

  addTask: (task: Task) => {
    set((state) => ({
      tasks: [...state.tasks, task],
    }))
  },

  updateTask: (task_id: string, updates: Partial<Task>) => {
    set((state) => ({
      tasks: state.tasks.map((task) =>
        task.task_id === task_id ? { ...task, ...updates } : task
      ),
    }))
  },

  removeTask: (task_id: string) => {
    set((state) => ({
      tasks: state.tasks.filter((task) => task.task_id !== task_id),
    }))
  },

  setCurrentTask: (task: Task | null) => {
    set({ currentTask: task })
  },

  clearTasks: () => {
    set({ tasks: [], currentTask: null })
  },
}))
