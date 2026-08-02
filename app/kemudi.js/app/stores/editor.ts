import { defineStore } from 'pinia'

export interface EditorState {
  nodes: { id: number; x: number; y: number; z: number }[]
  beams: { id: number; nodeA: number; nodeB: number }[]
  selectedNode: number | null
  selectedBeam: number | null
  hasUnsavedChanges: boolean
  undoStack: string[]
  redoStack: string[]
}

export const useEditorStore = defineStore('editor', {
  state: (): EditorState => ({
    nodes: [],
    beams: [],
    selectedNode: null,
    selectedBeam: null,
    hasUnsavedChanges: false,
    undoStack: [],
    redoStack: [],
  }),

  getters: {
    nodeCount: (state) => state.nodes.length,
    beamCount: (state) => state.beams.length,
    selection: (state) => ({
      node: state.nodes.find((n) => n.id === state.selectedNode) ?? null,
      beam: state.beams.find((b) => b.id === state.selectedBeam) ?? null,
    }),
  },

  actions: {
    addNode(x: number, y: number, z: number) {
      const id = this.nodes.length > 0 ? Math.max(...this.nodes.map((n) => n.id)) + 1 : 0
      this.saveUndo()
      this.nodes.push({ id, x, y, z })
      this.hasUnsavedChanges = true
      return id
    },

    removeNode(id: number) {
      this.saveUndo()
      this.nodes = this.nodes.filter((n) => n.id !== id)
      this.beams = this.beams.filter((b) => b.nodeA !== id && b.nodeB !== id)
      if (this.selectedNode === id) this.selectedNode = null
      this.hasUnsavedChanges = true
    },

    moveNode(id: number, x: number, y: number, z: number) {
      const node = this.nodes.find((n) => n.id === id)
      if (node) {
        this.saveUndo()
        node.x = x
        node.y = y
        node.z = z
        this.hasUnsavedChanges = true
      }
    },

    addBeam(nodeA: number, nodeB: number) {
      const id = this.beams.length > 0 ? Math.max(...this.beams.map((b) => b.id)) + 1 : 0
      this.saveUndo()
      this.beams.push({ id, nodeA, nodeB })
      this.hasUnsavedChanges = true
      return id
    },

    removeBeam(id: number) {
      this.saveUndo()
      this.beams = this.beams.filter((b) => b.id !== id)
      if (this.selectedBeam === id) this.selectedBeam = null
      this.hasUnsavedChanges = true
    },

    selectNode(id: number | null) {
      this.selectedNode = id
      this.selectedBeam = null
    },

    selectBeam(id: number | null) {
      this.selectedBeam = id
      this.selectedNode = null
    },

    saveUndo() {
      this.undoStack.push(JSON.stringify({ nodes: this.nodes, beams: this.beams }))
      this.redoStack = []
    },

    undo() {
      if (this.undoStack.length === 0) return
      const current = JSON.stringify({ nodes: this.nodes, beams: this.beams })
      this.redoStack.push(current)
      const prev = JSON.parse(this.undoStack.pop()!)
      this.nodes = prev.nodes
      this.beams = prev.beams
    },

    redo() {
      if (this.redoStack.length === 0) return
      const current = JSON.stringify({ nodes: this.nodes, beams: this.beams })
      this.undoStack.push(current)
      const next = JSON.parse(this.redoStack.pop()!)
      this.nodes = next.nodes
      this.beams = next.beams
    },

    resetEditor() {
      this.$reset()
    },

    $reset() {
      this.nodes = []
      this.beams = []
      this.selectedNode = null
      this.selectedBeam = null
      this.hasUnsavedChanges = false
      this.undoStack = []
      this.redoStack = []
    },
  },
})